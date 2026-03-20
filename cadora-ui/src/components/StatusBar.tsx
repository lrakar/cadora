import { type FC } from 'react';
import type { CursorInfo, TreeNode } from '../App';

interface StatusBarProps {
  cursorPos: CursorInfo;
  selectedNode: TreeNode | null;
}

const StatusBar: FC<StatusBarProps> = ({ cursorPos, selectedNode }) => {
  return (
    <div
      className="flex items-center h-6 px-3 text-[11px] select-none"
      style={{
        background: 'var(--bg-secondary)',
        borderTop: '1px solid var(--border-color)',
        color: 'var(--text-muted)',
      }}
    >
      {/* Coordinates */}
      <div className="flex items-center gap-3 mr-6">
        <span>
          <span style={{ color: '#ef4444' }}>X:</span>{' '}
          <span style={{ color: 'var(--text-primary)' }}>{cursorPos.x.toFixed(3)}</span>
        </span>
        <span>
          <span style={{ color: '#22c55e' }}>Y:</span>{' '}
          <span style={{ color: 'var(--text-primary)' }}>{cursorPos.y.toFixed(3)}</span>
        </span>
        <span>
          <span style={{ color: '#3b82f6' }}>Z:</span>{' '}
          <span style={{ color: 'var(--text-primary)' }}>{cursorPos.z.toFixed(3)}</span>
        </span>
      </div>

      {/* Separator */}
      <div className="w-px h-3 mx-2" style={{ background: 'var(--border-color)' }} />

      {/* Selection info */}
      <div className="flex-1">
        {selectedNode ? (
          <span>
            Selected: <span style={{ color: 'var(--text-primary)' }}>{selectedNode.label}</span>
            <span className="ml-1">({selectedNode.type})</span>
          </span>
        ) : (
          <span>No selection</span>
        )}
      </div>

      {/* Right side info */}
      <div className="flex items-center gap-4">
        <span>Units: mm</span>
        <span>Grid: 10mm</span>
      </div>
    </div>
  );
};

export default StatusBar;
