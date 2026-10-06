#!/usr/bin/env python3
"""
Cyclo's web front door, natively - the job orchestrator/ui/nginx.conf does
in Cyclo's container, on the same port (7080) and paths:

    /                         the built UI (orchestrator/ui/build), any unknown
                              path -> index.html (single-page app)
    /api/...                  -> supervisor API  127.0.0.1:7100
    /api/navigation/topics/ws -> its WebSocket
    /data-api/...             -> cyclo_data's video file server 127.0.0.1:7082
    /files/workspace/...      recordings etc. under /workspace (byte ranges)
    /urdf/...                 robot descriptions under shared/robot_configs

rosbridge (7090) and web_video_server (7085) are reached by the browser
directly, as in the container.

Run by orchestrator's cyclo_bringup.launch.py.
"""

import asyncio
import os
from pathlib import Path

import httpx
from starlette.applications import Starlette
from starlette.background import BackgroundTask
from starlette.requests import Request
from starlette.responses import FileResponse, Response, StreamingResponse
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocket, WebSocketDisconnect
import uvicorn
import websockets

CYCLO = Path(os.environ.get('CYCLO_DIR', Path(__file__).resolve().parents[1]))
UI_BUILD = CYCLO / 'orchestrator' / 'ui' / 'build'
URDF_DIR = CYCLO / 'shared' / 'shared' / 'robot_configs'
WORKSPACE = Path(os.environ.get('CYCLO_WORKSPACE', '/workspace'))  # set by the launch file
SUPERVISOR = 'http://127.0.0.1:' + os.environ.get('CYCLO_SUPERVISOR_API_PORT', '7100')
DATA_API = 'http://127.0.0.1:' + os.environ.get('CYCLO_VIDEO_SERVER_PORT', '7082')

HOP = {'connection', 'keep-alive', 'transfer-encoding', 'upgrade', 'host',
       'proxy-connection', 'te', 'trailer'}
NO_STORE = {'Cache-Control': 'no-store, no-cache, must-revalidate, proxy-revalidate'}
CORS = {'Access-Control-Allow-Origin': '*'}

client = httpx.AsyncClient(timeout=None)


async def _proxy(request: Request, base: str, read_s: float) -> Response:
    url = f'{base}/{request.path_params["path"]}'
    if request.url.query:
        url += f'?{request.url.query}'
    headers = {k: v for k, v in request.headers.items() if k.lower() not in HOP}
    headers['X-Real-IP'] = request.client.host if request.client else ''
    upstream = client.build_request(
        request.method, url, headers=headers, content=request.stream(),
        timeout=httpx.Timeout(connect=10.0, read=read_s, write=30.0, pool=10.0))
    try:
        resp = await client.send(upstream, stream=True)
    except httpx.HTTPError as e:
        return Response(f'upstream {base} not answering: {e}', status_code=502)
    out = {k: v for k, v in resp.headers.items() if k.lower() not in HOP}
    return StreamingResponse(resp.aiter_raw(), status_code=resp.status_code, headers=out,
                             background=BackgroundTask(resp.aclose))


async def api(request):
    path = request.path_params['path']
    return await _proxy(request, SUPERVISOR, 390.0 if path == 'navigation/goals/wait' else 150.0)


async def data_api(request):
    return await _proxy(request, DATA_API, 120.0)


async def topics_ws(ws: WebSocket):
    await ws.accept()
    url = SUPERVISOR.replace('http', 'ws', 1) + '/navigation/topics/ws'
    if ws.url.query:
        url += f'?{ws.url.query}'
    try:
        async with websockets.connect(url, max_size=None) as up:
            async def to_upstream():
                while True:
                    msg = await ws.receive()
                    if msg['type'] == 'websocket.disconnect':
                        return
                    if msg.get('text') is not None:
                        await up.send(msg['text'])
                    elif msg.get('bytes') is not None:
                        await up.send(msg['bytes'])

            async def to_browser():
                async for msg in up:
                    if isinstance(msg, str):
                        await ws.send_text(msg)
                    else:
                        await ws.send_bytes(msg)

            tasks = [asyncio.create_task(to_upstream()), asyncio.create_task(to_browser())]
            _, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for t in pending:
                t.cancel()
    except (OSError, websockets.WebSocketException, WebSocketDisconnect):
        pass
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass


def _file(base: Path, rel: str):
    """The file at base/rel, never outside base; None if there is none."""
    try:
        path = (base / rel).resolve()
        root = base.resolve()
    except OSError:
        return None
    if path != root and root not in path.parents:
        return None
    return path if path.is_file() else None


async def workspace_files(request):
    headers = {**CORS, 'Accept-Ranges': 'bytes', 'Cache-Control': 'no-cache',
               'Access-Control-Allow-Methods': 'GET, HEAD, OPTIONS',
               'Access-Control-Allow-Headers': 'Range',
               'Access-Control-Expose-Headers': 'Content-Length, Content-Range, Accept-Ranges'}
    if request.method == 'OPTIONS':
        return Response(status_code=204, headers=headers)
    path = _file(WORKSPACE, request.path_params['path'])
    if path is None:
        return Response('not found', status_code=404, headers=headers)
    media = {'.mcap': 'application/octet-stream', '.mp4': 'video/mp4'}.get(path.suffix)
    return FileResponse(path, headers=headers, media_type=media)


async def urdf(request):
    path = _file(URDF_DIR, request.path_params['path'])
    if path is None:
        return Response('not found', status_code=404)
    media = {'.urdf': 'application/xml', '.stl': 'model/stl'}.get(path.suffix)
    return FileResponse(path, media_type=media,
                        headers={**CORS, 'Cache-Control': 'public, max-age=86400'})


async def ui(request):
    rel = request.path_params['path']
    path = _file(UI_BUILD, rel) if rel else None
    if path is None:                                   # single-page app
        return FileResponse(UI_BUILD / 'index.html', headers=NO_STORE)
    if rel.startswith('static/'):
        return FileResponse(path, headers={'Cache-Control': 'public, max-age=31536000, immutable'})
    if path.suffix == '.wasm':
        return FileResponse(path, media_type='application/wasm',
                            headers={'Cache-Control': 'public, max-age=604800'})
    return FileResponse(path, headers=NO_STORE)


ALL = ['GET', 'HEAD', 'POST', 'PUT', 'PATCH', 'DELETE', 'OPTIONS']
app = Starlette(routes=[
    WebSocketRoute('/api/navigation/topics/ws', topics_ws),
    Route('/api/{path:path}', api, methods=ALL),
    Route('/data-api/{path:path}', data_api, methods=ALL),
    Route('/files/workspace/{path:path}', workspace_files, methods=['GET', 'HEAD', 'OPTIONS']),
    Route('/urdf/{path:path}', urdf, methods=['GET', 'HEAD']),
    Route('/{path:path}', ui, methods=['GET', 'HEAD']),
])


def main():
    if not (UI_BUILD / 'index.html').exists():
        raise SystemExit(f'no UI build in {UI_BUILD} - run cyclo_intelligence/native/install.sh')
    uvicorn.run(app, host=os.environ.get('CYCLO_UI_HOST', '0.0.0.0'),
                port=int(os.environ.get('CYCLO_UI_PORT', '7080')), log_level='warning')


if __name__ == '__main__':
    main()
