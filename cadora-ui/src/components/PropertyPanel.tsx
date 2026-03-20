import { type FC } from 'react';
import type { TreeNode } from '../App';

interface PropertyPanelProps {
  selectedNode: TreeNode | null;
}

interface PropertyRow {
  label: string;
  value: string;
}

function getProperties(node: TreeNode): PropertyRow[] {
  const base: PropertyRow[] = [
    { label: 'Name', value: node.label },
    { label: 'Type', value: node.type.charAt(0).toUpperCase() + node.type.slice(1) },
    { label: 'ID', value: node.id },
  ];

  switch (node.type) {
    case 'feature':
      return [
        ...base,
        { label: 'Length', value: '10.000 mm' },
        { label: 'Direction', value: 'Normal' },
        { label: 'Symmetric', value: 'No' },
      ];
    case 'sketch':
      return [
        ...base,
        { label: 'Support', value: 'XY Plane' },
        { label: 'Constraints', value: 'Fully constrained' },
        { label: 'Elements', value: '4' },
      ];
    case 'body':
      return [
        ...base,
        { label: 'Features', value: String(node.children?.length ?? 0) },
        { label: 'Visible', value: 'Yes' },
      ];
    case 'datum':
      return [
        ...base,
        { label: 'Placement', value: 'Origin' },
      ];
    default:
      return base;
  }
}

const PropertyPanel: FC<PropertyPanelProps> = ({ selectedNode }) => {
  return (
    <div
      className="flex flex-col overflow-y-auto select-none"
      style={{
        width: '260px',
        minWidth: '260px',
        background: 'var(--bg-panel)',
        borderLeft: '1px solid var(--border-color)',
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
        Properties
      </div>

      {/* Content */}
      <div className="p-3 flex-1 overflow-y-auto">
        {selectedNode ? (
          <div>
            {/* Object heading */}
            <div className="flex items-center mb-3 pb-2" style={{ borderBottom: '1px solid var(--border-subtle)' }}>
              <span className="text-sm font-medium" style={{ color: 'var(--text-primary)' }}>
                {selectedNode.label}
              </span>
              <span className="ml-2 text-[10px] px-1.5 py-0.5 rounded" style={{
                background: 'var(--bg-hover)',
                color: 'var(--text-muted)',
              }}>
                {selectedNode.type}
              </span>
            </div>

            {/* Properties table */}
            <div className="flex flex-col gap-1.5">
              {getProperties(selectedNode).map((prop) => (
                <div key={prop.label} className="flex items-center justify-between text-xs">
                  <span style={{ color: 'var(--text-muted)' }}>{prop.label}</span>
                  <span className="text-right ml-2 truncate max-w-[140px]" style={{ color: 'var(--text-primary)' }}>
                    {prop.value}
                  </span>
                </div>
              ))}
            </div>

            {/* Placement section */}
            {(selectedNode.type === 'feature' || selectedNode.type === 'body') && (
              <div className="mt-4">
                <div className="text-xs font-semibold mb-2" style={{ color: 'var(--text-secondary)' }}>
                  Placement
                </div>
                <div className="flex flex-col gap-1.5 text-xs">
                  <div className="flex items-center justify-between">
                    <span style={{ color: 'var(--text-muted)' }}>Position X</span>
                    <span style={{ color: 'var(--text-primary)' }}>0.000 mm</span>
                  </div>
                  <div className="flex items-center justify-between">
                    <span style={{ color: 'var(--text-muted)' }}>Position Y</span>
                    <span style={{ color: 'var(--text-primary)' }}>0.000 mm</span>
                  </div>
                  <div className="flex items-center justify-between">
                    <span style={{ color: 'var(--text-muted)' }}>Position Z</span>
                    <span style={{ color: 'var(--text-primary)' }}>0.000 mm</span>
                  </div>
                </div>
              </div>
            )}
          </div>
        ) : (
          <div className="text-xs text-center mt-8" style={{ color: 'var(--text-muted)' }}>
            Select an object to view properties
          </div>
        )}
      </div>
    </div>
  );
};

export default PropertyPanel;
