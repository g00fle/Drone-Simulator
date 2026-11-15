// Drone visual: the Onshape export (assets/drone.glb), with a procedural stand-in if it cannot load.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { DRONE, CAD_COM } from '../config/drone.js';

// Returns a group whose origin is the centre of mass, in the simulator body frame (x right, y up, -z nose).
export function createDroneVisual(url = './assets/drone.glb') {
  const group = new THREE.Group();
  const cadFrame = new THREE.Group(); // CAD is Z-up, +Y front
  cadFrame.rotation.x = -Math.PI / 2;
  cadFrame.position.set(-CAD_COM[0], -CAD_COM[2], CAD_COM[1]);
  group.add(cadFrame);
  const props = [];
  const fallback = buildFallback(props);
  cadFrame.add(fallback);
  const state = { group, props, loaded: false, info: 'Stand-in model (loading Onshape export...)' };

  const loader = new GLTFLoader();
  const failed = (err) => { state.info = 'Stand-in model (Onshape export failed to load)'; console.warn('drone model failed to load, using the stand-in model', err); };
  // Hosts that do not serve .glb get the same file as base64 inside JSON (assets/drone.glb.json).
  const loadPacked = () => fetch(url + '.json').then((r) => { if (!r.ok) throw new Error(`HTTP ${r.status}`); return r.json(); })
    .then((j) => loader.parse(Uint8Array.from(atob(j.glb), (c) => c.charCodeAt(0)).buffer, '', onLoad, failed)).catch(failed);
  loader.load(url, onLoad, undefined, loadPacked);

  function onLoad(gltf) {
    const scene = gltf.scene;
    const box = new THREE.Box3().setFromObject(scene), size = box.getSize(new THREE.Vector3());
    // Expect about 0.30 x 0.30 x 0.21 m. Fix millimetre exports.
    if (size.x > 50) { scene.scale.setScalar(0.001); size.multiplyScalar(0.001); }
    state.info = `Onshape model: ${(size.x * 1000).toFixed(0)} x ${(size.y * 1000).toFixed(0)} x ${(size.z * 1000).toFixed(0)} mm`;
    if (Math.abs(size.x - 0.3) > 0.03 || Math.abs(size.z - 0.21) > 0.04) console.warn('Drone model size differs from the plan (300 x 300 x 210 mm):', state.info);
    props.length = 0;
    const found = [];
    scene.traverse((o) => {
      if (o.isMesh) { o.castShadow = true; o.frustumCulled = false; }
      if (/^Propeller[ _]\d/.test(o.name || '')) found.push(o); // a Mesh, or a Group of primitives
    });
    for (const prop of found) { // vertices are baked in place: give each prop a pivot at its own centre
      const c = new THREE.Box3().setFromObject(prop).getCenter(new THREE.Vector3());
      prop.parent.worldToLocal(c);
      const pivot = new THREE.Group(); pivot.position.copy(c);
      prop.parent.add(pivot); pivot.add(prop); prop.position.sub(c);
      // the CAD prop is a solid disc: show it as a see-through blur, as a spinning prop looks
      prop.traverse((m) => { if (m.isMesh) { m.material = m.material.clone(); m.material.transparent = true; m.material.opacity = 0.22; m.material.depthWrite = false; m.castShadow = false; } });
      const idx = Number(prop.name.match(/Propeller[ _](\d)/)[1]) - 1;
      props.push({ pivot, motor: idx });
    }
    cadFrame.remove(fallback); cadFrame.add(scene); state.loaded = true;
    console.info(state.info, `(${found.length} propellers)`);
  }

  state.update = (quad, dt) => {
    for (const p of props) p.pivot.rotation.z += DRONE.motorSpin[p.motor % 4] * (8 + 110 * (quad.motors[p.motor % 4] || 0)) * dt;
  };
  return state;
}

// Simple model from the same dimensions as features/payloadQuadcopter.fs (CAD frame, metres).
function buildFallback(props) {
  const g = new THREE.Group();
  const dark = new THREE.MeshStandardMaterial({ color: 0x2a2d33, roughness: 0.6 }), accent = new THREE.MeshStandardMaterial({ color: 0xf07020, roughness: 0.6 });
  const D = 0.1125 * Math.SQRT1_2, LEG = 0.192 * Math.SQRT1_2;
  const add = (geo, mat, x, y, z, rz = 0, rx = 0) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.rotation.z = rz; m.rotation.x = rx; m.castShadow = true; g.add(m); return m; };
  add(new THREE.BoxGeometry(0.09, 0.13, 0.004), dark, 0, 0, 0.002);
  add(new THREE.BoxGeometry(0.05, 0.13, 0.004), dark, 0, 0, 0.042);
  add(new THREE.BoxGeometry(0.036, 0.075, 0.034), accent, 0, -0.002, 0.062); // battery
  add(new THREE.CylinderGeometry(0.0125, 0.0125, 0.02, 20), dark, 0, 0, -0.01, 0, Math.PI / 2); // magnet
  add(new THREE.BoxGeometry(0.02, 0.02, 0.02), accent, 0, 0.058, 0.058); // camera
  add(new THREE.BoxGeometry(0.006, 0.03, 0.006), dark, 0, 0.112, -0.08); // hook prong
  add(new THREE.BoxGeometry(0.006, 0.006, 0.08), dark, 0, 0.098, -0.04);
  [[1, 1], [-1, 1], [-1, -1], [1, -1]].forEach(([sx, sy], i) => {
    add(new THREE.BoxGeometry(0.02, 0.27, 0.008), dark, sx * LEG / 2, sy * LEG / 2, 0.008, -sx * sy * Math.PI / 4);
    add(new THREE.CylinderGeometry(0.014, 0.014, 0.02, 16), accent, sx * D, sy * D, 0.02, 0, Math.PI / 2);
    add(new THREE.TorusGeometry(0.069, 0.003, 6, 32), sy > 0 ? accent : dark, sx * D, sy * D, 0.032);
    add(new THREE.BoxGeometry(0.01, 0.01, 0.13), dark, sx * LEG, sy * LEG, -0.065);
    const pivot = new THREE.Group(); pivot.position.set(sx * D, sy * D, 0.034); g.add(pivot);
    const blade = new THREE.Mesh(new THREE.BoxGeometry(0.127, 0.014, 0.002), new THREE.MeshStandardMaterial({ color: 0x9fd3ff, transparent: true, opacity: 0.7 }));
    pivot.add(blade); props.push({ pivot, motor: i });
  });
  return g;
}
