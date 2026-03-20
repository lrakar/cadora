import { type FC, useRef, useEffect, useCallback } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { CursorInfo } from '../App';

interface Viewport3DProps {
  selectedId: string | null;
  onCursorMove: (pos: CursorInfo) => void;
}

const Viewport3D: FC<Viewport3DProps> = ({ onCursorMove }) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const raycasterRef = useRef(new THREE.Raycaster());
  const mouseRef = useRef(new THREE.Vector2());
  const animFrameRef = useRef<number>(0);

  const initScene = useCallback(() => {
    if (!containerRef.current) return;
    const container = containerRef.current;

    // Scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x4a4a4a);
    sceneRef.current = scene;

    // Camera
    const camera = new THREE.PerspectiveCamera(
      45,
      container.clientWidth / container.clientHeight,
      0.1,
      10000
    );
    camera.position.set(30, 25, 30);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Controls (Fusion 360 style: orbit, pan, zoom)
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.rotateSpeed = 0.8;
    controls.panSpeed = 0.8;
    controls.zoomSpeed = 1.2;
    controls.target.set(0, 0, 0);
    controls.update();
    controlsRef.current = controls;

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(50, 80, 50);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    scene.add(dirLight);

    const fillLight = new THREE.DirectionalLight(0xffffff, 0.3);
    fillLight.position.set(-30, 20, -30);
    scene.add(fillLight);

    // Ground grid
    const gridHelper = new THREE.GridHelper(100, 100, 0x666666, 0x555555);
    scene.add(gridHelper);

    // Axis indicator
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    // Demo cube (placeholder for CAD geometry)
    const geometry = new THREE.BoxGeometry(10, 10, 10);
    const material = new THREE.MeshPhongMaterial({
      color: 0x6699cc,
      specular: 0x222222,
      shininess: 30,
      flatShading: false,
    });
    const cube = new THREE.Mesh(geometry, material);
    cube.position.set(0, 5, 0);
    cube.castShadow = true;
    cube.receiveShadow = true;
    cube.userData.selectable = true;
    scene.add(cube);

    // Edge wireframe on the cube
    const edges = new THREE.EdgesGeometry(geometry);
    const lineMaterial = new THREE.LineBasicMaterial({ color: 0x333333 });
    const wireframe = new THREE.LineSegments(edges, lineMaterial);
    wireframe.position.copy(cube.position);
    scene.add(wireframe);

    // Ground plane (for shadow receiving)
    const planeGeo = new THREE.PlaneGeometry(100, 100);
    const planeMat = new THREE.ShadowMaterial({ opacity: 0.15 });
    const plane = new THREE.Mesh(planeGeo, planeMat);
    plane.rotation.x = -Math.PI / 2;
    plane.receiveShadow = true;
    scene.add(plane);

    // Animation loop
    const animate = () => {
      animFrameRef.current = requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();
  }, []);

  // Mouse move for coordinate display
  const handleMouseMove = useCallback((event: MouseEvent) => {
    if (!containerRef.current || !cameraRef.current || !sceneRef.current) return;

    const rect = containerRef.current.getBoundingClientRect();
    mouseRef.current.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouseRef.current.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    // Raycast to ground plane (y=0)
    raycasterRef.current.setFromCamera(mouseRef.current, cameraRef.current);
    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const intersection = new THREE.Vector3();
    raycasterRef.current.ray.intersectPlane(groundPlane, intersection);

    if (intersection) {
      onCursorMove({
        x: Math.round(intersection.x * 1000) / 1000,
        y: Math.round(intersection.y * 1000) / 1000,
        z: Math.round(intersection.z * 1000) / 1000,
      });
    }
  }, [onCursorMove]);

  // Resize handler
  const handleResize = useCallback(() => {
    if (!containerRef.current || !rendererRef.current || !cameraRef.current) return;
    const { clientWidth, clientHeight } = containerRef.current;
    cameraRef.current.aspect = clientWidth / clientHeight;
    cameraRef.current.updateProjectionMatrix();
    rendererRef.current.setSize(clientWidth, clientHeight);
  }, []);

  useEffect(() => {
    initScene();

    const container = containerRef.current;
    if (container) {
      container.addEventListener('mousemove', handleMouseMove);
    }

    const resizeObserver = new ResizeObserver(handleResize);
    if (container) {
      resizeObserver.observe(container);
    }

    return () => {
      if (animFrameRef.current) {
        cancelAnimationFrame(animFrameRef.current);
      }
      if (container) {
        container.removeEventListener('mousemove', handleMouseMove);
        resizeObserver.unobserve(container);
      }
      if (rendererRef.current) {
        rendererRef.current.dispose();
        if (container && rendererRef.current.domElement.parentNode === container) {
          container.removeChild(rendererRef.current.domElement);
        }
      }
      controlsRef.current?.dispose();
    };
  }, [initScene, handleMouseMove, handleResize]);

  return (
    <div
      ref={containerRef}
      className="w-full h-full"
      style={{ background: 'var(--bg-viewport)' }}
    />
  );
};

export default Viewport3D;
