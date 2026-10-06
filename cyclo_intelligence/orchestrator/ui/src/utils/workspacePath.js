export const DEFAULT_WORKSPACE_MOUNT = {
  container_root: '/workspace',
  host_root: '',
  host_available: false,
};

const stripTrailingSlash = (value) => (value || '').trim().replace(/\/+$/, '');

export const mapWorkspacePathToHost = (containerPath, workspaceMount) => {
  const normalizedPath = stripTrailingSlash(containerPath);
  const containerRoot = stripTrailingSlash(
    workspaceMount?.container_root || DEFAULT_WORKSPACE_MOUNT.container_root
  );
  const hostRoot = stripTrailingSlash(workspaceMount?.host_root || '');

  if (!normalizedPath || !containerRoot || !hostRoot) return '';
  if (normalizedPath !== containerRoot && !normalizedPath.startsWith(`${containerRoot}/`)) {
    return '';
  }

  const relativePath = normalizedPath.slice(containerRoot.length).replace(/^\/+/, '');
  return relativePath ? `${hostRoot}/${relativePath}` : hostRoot;
};
