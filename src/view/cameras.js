// FPV (from the drone's camera), line-of-sight from the Pilot Area, chase, and top-down for the editor.
import * as THREE from 'three';
import { DRONE } from '../config/drone.js';
import { FIELD } from '../config/course.js';

export const VIEWS = ['fpv', 'los', 'chase'];
export const VIEW_NAMES = { fpv: 'FPV camera', los: 'Line of sight', chase: 'Chase', top: 'Layout editor' };

export function createCameras(droneGroup) {
  const cam = DRONE.camera;
  const fpv = new THREE.PerspectiveCamera(90, 1, 0.01, 60);
  fpv.position.set(...cam.pos); fpv.rotation.x = (cam.tiltUpDeg * Math.PI) / 180;
  droneGroup.add(fpv);
  const los = new THREE.PerspectiveCamera(50, 1, 0.05, 80);
  los.position.set(...FIELD.pilotEye);
  const chase = new THREE.PerspectiveCamera(70, 1, 0.02, 80);
  const top = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 50);
  const zc = FIELD.pilotDepth / 2;
  top.position.set(0, 20, zc); top.up.set(0, 0, -1); top.lookAt(0, 0, zc);
  const look = new THREE.Vector3(0, 0.5, 0), want = new THREE.Vector3(), fwd = new THREE.Vector3();
  let first = true;
  return {
    fpv, los, chase, top,
    get(view) { return { fpv, los, chase, top }[view]; },
    resize(aspect) {
      // the configured FOV is horizontal, as FPV cameras are specified
      const h = (cam.fovDeg * Math.PI) / 180;
      fpv.fov = (2 * Math.atan(Math.tan(h / 2) / Math.max(aspect, 1)) * 180) / Math.PI;
      for (const c of [fpv, los, chase]) { c.aspect = aspect; c.updateProjectionMatrix(); }
      // fit the whole field (length is vertical on screen)
      let halfH = (FIELD.length + FIELD.pilotDepth) / 2 + 0.4, halfW = halfH * aspect;
      const needW = FIELD.width / 2 + 0.6;
      if (halfW < needW) { halfW = needW; halfH = halfW / aspect; }
      top.left = -halfW; top.right = halfW; top.top = halfH; top.bottom = -halfH; top.updateProjectionMatrix();
    },
    update(dt) {
      const p = droneGroup.position;
      look.lerp(p, first ? 1 : Math.min(1, dt * 8));
      los.lookAt(look);
      const d = los.position.distanceTo(p);
      los.fov = THREE.MathUtils.clamp((2 * Math.atan(1.3 / d) * 180) / Math.PI, 16, 50); los.updateProjectionMatrix();
      fwd.set(0, 0, -1).applyQuaternion(droneGroup.quaternion); fwd.y = 0; fwd.normalize();
      want.copy(p).addScaledVector(fwd, -0.9); want.y += 0.4;
      chase.position.lerp(want, first ? 1 : Math.min(1, dt * 5)); chase.lookAt(p.x, p.y + 0.05, p.z);
      first = false;
    },
  };
}
