import { type FC, useState } from 'react';
import type { TreeNode } from '../App';

interface ModelTreeProps {
  tree: TreeNode[];
  selectedId: string | null;
  onSelect: (id: string) => void;
}

const typeIcons: Record<string, string> = {
  origin: '⊕',
  body: '📦',
  feature: '🔧',
  sketch: '✏️',
  datum: '▬',
};

interface TreeItemProps {
  node: TreeNode;
  depth: number;
  selectedId: string | null;
  onSelect: (id: string) => void;
}

const TreeItem: FC<TreeItemProps> = ({ node, depth, selectedId, onSelect }) => {
  const [expanded, setExpanded] = useState(true);
  const hasChildren = node.children && node.children.length > 0;
  const isSelected = node.id === selectedId;

  return (
    <div>
      <div
        className="flex items-center py-0.5 px-1 cursor-pointer rounded-sm transition-colors"
        style={{
          paddingLeft: `${depth * 16 + 4}px`,
          background: isSelected ? 'var(--accent)' : 'transparent',
          color: isSelected ? '#fff' : 'var(--text-primary)',
        }}
        onClick={() => onSelect(node.id)}
        onDoubleClick={() => hasChildren && setExpanded(!expanded)}
        onMouseEnter={(e) => {
          if (!isSelected) e.currentTarget.style.background = 'var(--bg-hover)';
        }}
        onMouseLeave={(e) => {
          if (!isSelected) e.currentTarget.style.background = 'transparent';
        }}
      >
        {hasChildren ? (
          <span
            className="w-4 text-center text-[10px] mr-0.5 flex-shrink-0 cursor-pointer"
            style={{ color: isSelected ? '#fff' : 'var(--text-muted)' }}
            onClick={(e) => { e.stopPropagation(); setExpanded(!expanded); }}
          >
            {expanded ? '▼' : '▶'}
          </span>
        ) : (
          <span className="w-4 mr-0.5 flex-shrink-0" />
        )}
        <span className="mr-1.5 text-xs">{typeIcons[node.type] || '•'}</span>
        <span className="text-xs truncate">{node.label}</span>
      </div>
      {hasChildren && expanded && (
        <div>
          {node.children!.map((child) => (
            <TreeItem
              key={child.id}
              node={child}
              depth={depth + 1}
              selectedId={selectedId}
              onSelect={onSelect}
            />
          ))}
        </div>
      )}
    </div>
  );
};

const ModelTree: FC<ModelTreeProps> = ({ tree, selectedId, onSelect }) => {
  return (
    <div
      className="flex flex-col overflow-y-auto select-none"
      style={{
        width: '240px',
        minWidth: '240px',
        background: 'var(--bg-panel)',
        borderRight: '1px solid var(--border-color)',
      }}
    >
      {/* Header */}
      <div
        className="flex items-center h-7 px-3 text-xs font-semibold uppercase tracking-wider"
        style={{
          background: 'var(--bg-secondary)',
          color: 'var(--text-secondary)',
          borderBottom: '1px solid var(--border-subtle)',
        }}
      >
        Browser
      </div>

      {/* Tree content */}
      <div className="py-1 flex-1 overflow-y-auto">
        {tree.map((node) => (
          <TreeItem
            key={node.id}
            node={node}
            depth={0}
            selectedId={selectedId}
            onSelect={onSelect}
          />
        ))}
      </div>
    </div>
  );
};

export default ModelTree;
