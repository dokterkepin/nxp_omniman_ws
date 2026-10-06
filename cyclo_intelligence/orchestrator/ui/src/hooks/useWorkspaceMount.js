import { useEffect, useState } from 'react';
import { DEFAULT_WORKSPACE_MOUNT } from '../utils/workspacePath';

const API_BASE = '/api';

export default function useWorkspaceMount() {
  const [workspaceMount, setWorkspaceMount] = useState(DEFAULT_WORKSPACE_MOUNT);

  useEffect(() => {
    let cancelled = false;

    const loadWorkspaceMount = async () => {
      try {
        const response = await fetch(`${API_BASE}/workspace`);
        if (!response.ok) return;
        const data = await response.json();
        if (!cancelled) {
          setWorkspaceMount({
            ...DEFAULT_WORKSPACE_MOUNT,
            ...data,
          });
        }
      } catch (error) {
        console.debug('Workspace mount lookup failed:', error.message);
      }
    };

    loadWorkspaceMount();
    return () => {
      cancelled = true;
    };
  }, []);

  return workspaceMount;
}
