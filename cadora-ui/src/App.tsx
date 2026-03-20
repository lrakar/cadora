import { useState, useCallback } from 'react';
import Toolbar from './components/Toolbar';
import ModelTree from './components/ModelTree';
import Viewport3D from './components/Viewport3D';
import PropertyPanel from './components/PropertyPanel';
import StatusBar from './components/StatusBar';

export interface TreeNode {
  id: string;
  label: string;
  type: 'body' | 'feature' | 'sketch' | 'datum' | 'origin';
  children?: TreeNode[];
  visible?: boolean;
  icon?: string;
}

export interface CursorInfo {
  x: number;
  y: number;
  z: number;
}

const defaultTree: TreeNode[] = [
  {
    id: 'origin',
    label: 'Origin',
    type: 'origin',
    children: [
      { id: 'xy-plane', label: 'XY Plane', type: 'datum' },
      { id: 'xz-plane', label: 'XZ Plane', type: 'datum' },
      { id: 'yz-plane', label: 'YZ Plane', type: 'datum' },
      { id: 'x-axis', label: 'X Axis', type: 'datum' },
      { id: 'y-axis', label: 'Y Axis', type: 'datum' },
      { id: 'z-axis', label: 'Z Axis', type: 'datum' },
    ],
  },
  {
    id: 'body1',
    label: 'Body',
    type: 'body',
    children: [
      { id: 'sketch1', label: 'Sketch', type: 'sketch' },
      { id: 'pad1', label: 'Pad', type: 'feature' },
    ],
  },
];

function App() {
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [tree] = useState<TreeNode[]>(defaultTree);
  const [cursorPos, setCursorPos] = useState<CursorInfo>({ x: 0, y: 0, z: 0 });
  const [activeTab, setActiveTab] = useState('model');

  const handleSelect = useCallback((id: string) => {
    setSelectedId(id);
  }, []);

  const handleCursorMove = useCallback((pos: CursorInfo) => {
    setCursorPos(pos);
  }, []);

  const selectedNode = findNode(tree, selectedId);

  return (
    <div className="flex flex-col w-full h-full">
      <Toolbar activeTab={activeTab} onTabChange={setActiveTab} />

      <div className="flex flex-1 overflow-hidden">
        <ModelTree
          tree={tree}
          selectedId={selectedId}
          onSelect={handleSelect}
        />

        <div className="flex-1 relative">
          <Viewport3D
            selectedId={selectedId}
            onCursorMove={handleCursorMove}
          />
        </div>

        <PropertyPanel selectedNode={selectedNode} />
      </div>

      <StatusBar cursorPos={cursorPos} selectedNode={selectedNode} />
    </div>
  );
}

function findNode(nodes: TreeNode[], id: string | null): TreeNode | null {
  if (!id) return null;
  for (const node of nodes) {
    if (node.id === id) return node;
    if (node.children) {
      const found = findNode(node.children, id);
      if (found) return found;
    }
  }
  return null;
}

export default App;
