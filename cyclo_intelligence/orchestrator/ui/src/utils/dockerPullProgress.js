export function parseDockerPullSseBlock(block) {
  const lines = String(block || '').split(/\r?\n/);
  let event = 'message';
  const dataLines = [];

  for (const line of lines) {
    if (line.startsWith('event:')) {
      event = line.slice('event:'.length).trim() || 'message';
    } else if (line.startsWith('data:')) {
      dataLines.push(line.slice('data:'.length).trimStart());
    }
  }

  if (dataLines.length === 0) return null;

  const rawData = dataLines.join('\n');
  try {
    return { event, data: JSON.parse(rawData) };
  } catch {
    return { event, data: { message: rawData } };
  }
}

function pullMessage(chunk) {
  if (!chunk) return 'Preparing image pull...';
  const message = chunk.error || chunk.message || chunk.status || 'Pulling image...';
  return chunk.id ? `${chunk.id}: ${message}` : message;
}

function layerPercentFrom(chunk) {
  const detail = chunk?.progressDetail || {};
  const current = Number(detail.current);
  const total = Number(detail.total);
  if (!Number.isFinite(current) || !Number.isFinite(total) || total <= 0) {
    return null;
  }
  return Math.max(0, Math.min(100, Math.round((current / total) * 100)));
}

function isLayerReady(status) {
  return status === 'Already exists' || status === 'Pull complete';
}

function layerDetail(completedLayers, layers) {
  if (!layers) return '';
  return `${completedLayers}/${layers} layers ready`;
}

export function createDockerPullProgressTracker() {
  const layers = new Map();

  const update = (chunk) => {
    const layerPercent = layerPercentFrom(chunk);
    const status = chunk?.status || '';

    if (chunk?.id) {
      const previous = layers.get(chunk.id) || {};
      layers.set(chunk.id, {
        ...previous,
        status,
        ready: previous.ready || isLayerReady(status),
      });
    }

    let completedLayers = 0;
    layers.forEach((layer) => {
      if (layer.ready) completedLayers += 1;
    });

    const message = pullMessage(chunk);

    return {
      percent: null,
      layerPercent,
      message: layerPercent === null ? message : `${message} ${layerPercent}%`,
      layers: layers.size,
      completedLayers,
      detail: layerDetail(completedLayers, layers.size),
    };
  };

  const complete = (message = 'Image pull complete') => ({
    percent: 100,
    layerPercent: null,
    message,
    layers: layers.size,
    completedLayers: layers.size,
    detail: layers.size ? layerDetail(layers.size, layers.size) : '',
  });

  return { update, complete };
}
