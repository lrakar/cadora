import { type FC } from 'react';

interface ToolbarProps {
  activeTab: string;
  onTabChange: (tab: string) => void;
}

const tabs = [
  { id: 'model', label: 'MODEL' },
  { id: 'sketch', label: 'SKETCH' },
  { id: 'construct', label: 'CONSTRUCT' },
  { id: 'inspect', label: 'INSPECT' },
  { id: 'insert', label: 'INSERT' },
];

interface ToolButton {
  id: string;
  label: string;
  icon: string;
}

const modelTools: ToolButton[] = [
  { id: 'new-body', label: 'New Body', icon: '📦' },
  { id: 'new-sketch', label: 'Sketch', icon: '✏️' },
  { id: 'pad', label: 'Pad', icon: '⬆️' },
  { id: 'pocket', label: 'Pocket', icon: '⬇️' },
  { id: 'revolve', label: 'Revolve', icon: '🔄' },
  { id: 'fillet', label: 'Fillet', icon: '◐' },
  { id: 'chamfer', label: 'Chamfer', icon: '◢' },
  { id: 'mirror', label: 'Mirror', icon: '🪞' },
  { id: 'pattern', label: 'Pattern', icon: '⊞' },
];

const sketchTools: ToolButton[] = [
  { id: 'line', label: 'Line', icon: '╱' },
  { id: 'rect', label: 'Rectangle', icon: '▭' },
  { id: 'circle', label: 'Circle', icon: '○' },
  { id: 'arc', label: 'Arc', icon: '◠' },
  { id: 'trim', label: 'Trim', icon: '✂️' },
  { id: 'dimension', label: 'Dimension', icon: '📏' },
  { id: 'constrain', label: 'Constrain', icon: '🔒' },
  { id: 'finish', label: 'Finish', icon: '✓' },
];

const constructTools: ToolButton[] = [
  { id: 'datum-plane', label: 'Datum Plane', icon: '▬' },
  { id: 'datum-axis', label: 'Datum Axis', icon: '│' },
  { id: 'datum-point', label: 'Datum Point', icon: '•' },
];

const inspectTools: ToolButton[] = [
  { id: 'measure', label: 'Measure', icon: '📐' },
  { id: 'section', label: 'Section', icon: '⊟' },
];

const insertTools: ToolButton[] = [
  { id: 'import-step', label: 'STEP', icon: '📥' },
  { id: 'import-stl', label: 'STL', icon: '📥' },
  { id: 'import-iges', label: 'IGES', icon: '📥' },
  { id: 'export-step', label: 'Export STEP', icon: '📤' },
  { id: 'export-stl', label: 'Export STL', icon: '📤' },
];

function getToolsForTab(tab: string): ToolButton[] {
  switch (tab) {
    case 'model': return modelTools;
    case 'sketch': return sketchTools;
    case 'construct': return constructTools;
    case 'inspect': return inspectTools;
    case 'insert': return insertTools;
    default: return modelTools;
  }
}

const Toolbar: FC<ToolbarProps> = ({ activeTab, onTabChange }) => {
  const tools = getToolsForTab(activeTab);

  return (
    <div className="flex flex-col select-none" style={{ background: 'var(--bg-toolbar)' }}>
      {/* Tab bar */}
      <div className="flex items-center h-8 px-2 gap-1" style={{ background: 'var(--bg-secondary)', borderBottom: '1px solid var(--border-subtle)' }}>
        <span className="font-bold text-sm mr-4 tracking-wider" style={{ color: 'var(--accent)' }}>CADORA</span>
        {tabs.map((tab) => (
          <button
            key={tab.id}
            onClick={() => onTabChange(tab.id)}
            className="px-3 py-1 text-xs font-medium rounded-sm transition-colors cursor-pointer"
            style={{
              background: activeTab === tab.id ? 'var(--bg-active)' : 'transparent',
              color: activeTab === tab.id ? '#fff' : 'var(--text-secondary)',
            }}
            onMouseEnter={(e) => {
              if (activeTab !== tab.id) e.currentTarget.style.background = 'var(--bg-hover)';
            }}
            onMouseLeave={(e) => {
              if (activeTab !== tab.id) e.currentTarget.style.background = 'transparent';
            }}
          >
            {tab.label}
          </button>
        ))}

        {/* Right side: file operations */}
        <div className="ml-auto flex gap-1">
          <button className="px-2 py-1 text-xs rounded-sm cursor-pointer" style={{ color: 'var(--text-secondary)' }}
            onMouseEnter={(e) => e.currentTarget.style.background = 'var(--bg-hover)'}
            onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
            title="New">📄 New</button>
          <button className="px-2 py-1 text-xs rounded-sm cursor-pointer" style={{ color: 'var(--text-secondary)' }}
            onMouseEnter={(e) => e.currentTarget.style.background = 'var(--bg-hover)'}
            onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
            title="Open">📂 Open</button>
          <button className="px-2 py-1 text-xs rounded-sm cursor-pointer" style={{ color: 'var(--text-secondary)' }}
            onMouseEnter={(e) => e.currentTarget.style.background = 'var(--bg-hover)'}
            onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
            title="Save">💾 Save</button>
          <button className="px-2 py-1 text-xs rounded-sm cursor-pointer" style={{ color: 'var(--text-secondary)' }}
            onMouseEnter={(e) => e.currentTarget.style.background = 'var(--bg-hover)'}
            onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
            title="Undo">↩️</button>
          <button className="px-2 py-1 text-xs rounded-sm cursor-pointer" style={{ color: 'var(--text-secondary)' }}
            onMouseEnter={(e) => e.currentTarget.style.background = 'var(--bg-hover)'}
            onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
            title="Redo">↪️</button>
        </div>
      </div>

      {/* Tool buttons */}
      <div className="flex items-center h-12 px-2 gap-1" style={{ borderBottom: '1px solid var(--border-color)' }}>
        {tools.map((tool) => (
          <button
            key={tool.id}
            className="flex flex-col items-center justify-center px-3 py-1 rounded-sm transition-colors cursor-pointer min-w-[56px]"
            style={{ color: 'var(--text-primary)' }}
            onMouseEnter={(e) => e.currentTarget.style.background = 'var(--bg-hover)'}
            onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
            title={tool.label}
          >
            <span className="text-base leading-none">{tool.icon}</span>
            <span className="text-[10px] mt-0.5 whitespace-nowrap" style={{ color: 'var(--text-secondary)' }}>{tool.label}</span>
          </button>
        ))}
      </div>
    </div>
  );
};

export default Toolbar;
