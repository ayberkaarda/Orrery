// Hero motif: a mechanical orrery drawn with three.js.
//
// The model (orbits paced by Kepler's third law) lives in orrery-model.js.
// Each planet rides on a brass arm from the central column, as on a
// mechanical orrery. Decorative: the screenshots further down the page come
// from the application itself.
//
// The scene is interactive. Pointing at a planet highlights it, a click or tap
// follows it with the camera, a drag turns the camera and the wheel or a pinch
// zooms. This module never touches the DOM, so it runs in a worker on an
// OffscreenCanvas where available (orrery-worker.js) and on the main thread
// otherwise; site.js owns the DOM side: events, labels and announcements.

import * as THREE from '../vendor/three/three.module.min.js';
import { PLANETS, ZOOM_MAX, ZOOM_MIN, omega } from './orrery-model.js';

const BRASS = 0xe2a75c;
const STEEL = 0x74aacc;
const ORBIT = 0x8e96a4;

const SCALE_RING = 5.75;
const TILT = THREE.MathUtils.degToRad(23); // default camera elevation above the plane
const PITCH_MIN = THREE.MathUtils.degToRad(4);
const PITCH_MAX = THREE.MathUtils.degToRad(72);
const FOCUS_ZOOM = 0.62;
const IDLE_SPIN = 0.02; // rad/s of idle camera rotation
const IDLE_DELAY = 2500; // ms after the last interaction before idle rotation resumes
const PICK_INTERVAL = 40; // ms between pointer picks
const PICK_SLOP = 16; // px added to a planet's projected radius when picking
const DRAG_THRESHOLD = 5; // px before a press becomes a drag
const CENTRE = new THREE.Vector3(0, -0.35, 0);

// Works in a worker (OffscreenCanvas) and on the main thread.
function makeCanvas(size) {
  if (typeof OffscreenCanvas !== 'undefined') return new OffscreenCanvas(size, size);
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  return canvas;
}

function radialTexture(size, stops) {
  const canvas = makeCanvas(size);
  const g = canvas.getContext('2d');
  const grad = g.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  for (const [at, colour] of stops) grad.addColorStop(at, colour);
  g.fillStyle = grad;
  g.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function circlePoints(radius, segments, y = 0) {
  const points = [];
  for (let i = 0; i < segments; i += 1) {
    const t = (i / segments) * Math.PI * 2;
    points.push(new THREE.Vector3(Math.cos(t) * radius, y, Math.sin(t) * radius));
  }
  return points;
}

function lineMaterial(color, opacity) {
  return new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthWrite: false });
}

function buildScene() {
  const scene = new THREE.Scene();
  const disposables = [];
  const track = (obj) => {
    disposables.push(obj);
    return obj;
  };

  // Light from the sun, plus a faint fill so night sides keep their shape.
  scene.add(new THREE.PointLight(0xffe2b8, 3.2, 0, 0));
  scene.add(new THREE.AmbientLight(0x6a7890, 0.35));

  // Sun and its glow.
  const sun = new THREE.Mesh(
    track(new THREE.SphereGeometry(0.3, 48, 24)),
    track(new THREE.MeshBasicMaterial({ color: 0xffd7a0 })),
  );
  scene.add(sun);
  const glowMap = track(
    radialTexture(128, [
      [0, 'rgba(255, 236, 205, 1)'],
      [0.18, 'rgba(255, 206, 140, 0.55)'],
      [0.45, 'rgba(226, 167, 92, 0.14)'],
      [1, 'rgba(226, 167, 92, 0)'],
    ]),
  );
  const glow = new THREE.Sprite(
    track(
      new THREE.SpriteMaterial({
        map: glowMap,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
        transparent: true,
      }),
    ),
  );
  glow.scale.setScalar(2.9);
  scene.add(glow);

  // Halo used to highlight a planet; tinted per planet.
  const haloMap = track(
    radialTexture(64, [
      [0, 'rgba(255, 255, 255, 0.9)'],
      [0.3, 'rgba(255, 255, 255, 0.35)'],
      [1, 'rgba(255, 255, 255, 0)'],
    ]),
  );

  // Central column under the sun and the drum the arms turn on.
  const columnMat = track(lineMaterial(BRASS, 0.5));
  const column = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, -0.3, 0),
    new THREE.Vector3(0, -1.6, 0),
  ]);
  scene.add(new THREE.Line(track(column), columnMat));

  // Graduated scale ring: a tick every 5 degrees, a long one every 30.
  const scaleMat = track(lineMaterial(BRASS, 0.32));
  scene.add(
    new THREE.LineLoop(
      track(new THREE.BufferGeometry().setFromPoints(circlePoints(SCALE_RING, 256))),
      scaleMat,
    ),
  );
  const tickPoints = [];
  for (let deg = 0; deg < 360; deg += 5) {
    const t = THREE.MathUtils.degToRad(deg);
    const len = deg % 30 === 0 ? 0.3 : 0.12;
    const c = Math.cos(t);
    const s = Math.sin(t);
    tickPoints.push(new THREE.Vector3(c * SCALE_RING, 0, s * SCALE_RING));
    tickPoints.push(new THREE.Vector3(c * (SCALE_RING + len), 0, s * (SCALE_RING + len)));
  }
  scene.add(
    new THREE.LineSegments(track(new THREE.BufferGeometry().setFromPoints(tickPoints)), scaleMat),
  );

  // Orbits, planets and arms.
  const armMat = track(lineMaterial(BRASS, 0.55));
  const bodies = PLANETS.map((p, i) => {
    const orbitMat = track(lineMaterial(ORBIT, 0.24));
    scene.add(
      new THREE.LineLoop(
        track(new THREE.BufferGeometry().setFromPoints(circlePoints(p.a, 192))),
        orbitMat,
      ),
    );

    const pivot = new THREE.Group();
    scene.add(pivot);

    const planet = new THREE.Mesh(
      track(new THREE.SphereGeometry(p.r, 32, 16)),
      track(new THREE.MeshLambertMaterial({ color: p.color })),
    );
    planet.position.set(p.a, 0, 0);
    pivot.add(planet);

    const haloMat = track(
      new THREE.SpriteMaterial({
        map: haloMap,
        color: p.color,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
        transparent: true,
        opacity: 0,
      }),
    );
    const halo = new THREE.Sprite(haloMat);
    halo.scale.setScalar(Math.max(p.r * 7, 0.5));
    halo.visible = false;
    planet.add(halo);

    // Arm: horizontal at its own level below the plane, then up to the body.
    const level = -0.42 - i * 0.17;
    const arm = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, level, 0),
      new THREE.Vector3(p.a, level, 0),
      new THREE.Vector3(p.a, -p.r, 0),
    ]);
    pivot.add(new THREE.Line(track(arm), armMat));

    if (p.ring) {
      const ring = new THREE.Mesh(
        track(new THREE.RingGeometry(p.r * 1.45, p.r * 2.2, 64)),
        track(
          new THREE.MeshBasicMaterial({
            color: 0xd8c09a,
            transparent: true,
            opacity: 0.4,
            side: THREE.DoubleSide,
            depthWrite: false,
          }),
        ),
      );
      ring.rotation.x = -Math.PI / 2 + 0.45;
      planet.add(ring);
    }

    let moonPivot = null;
    if (p.moon) {
      moonPivot = new THREE.Group();
      moonPivot.position.set(p.a, 0, 0);
      pivot.add(moonPivot);
      const moonOrbit = new THREE.LineLoop(
        track(new THREE.BufferGeometry().setFromPoints(circlePoints(0.3, 64))),
        track(lineMaterial(STEEL, 0.3)),
      );
      moonPivot.add(moonOrbit);
      const moon = new THREE.Mesh(
        track(new THREE.SphereGeometry(0.035, 16, 8)),
        track(new THREE.MeshLambertMaterial({ color: 0xd4d6da })),
      );
      moon.position.set(0.3, 0, 0);
      moonPivot.add(moon);
    }

    return {
      pivot,
      moonPivot,
      planet,
      halo,
      haloMat,
      orbitMat,
      a: p.a,
      r: p.r,
      phase: p.phase,
      omega: omega(i),
      glow: 0, // eased highlight, 0..1
      world: new THREE.Vector3(),
    };
  });

  // Background stars: fixed seed, so every visit shows the same sky.
  let seed = 1977;
  const random = () => {
    seed = (seed * 16807) % 2147483647;
    return seed / 2147483647;
  };
  const starCount = 700;
  const positions = new Float32Array(starCount * 3);
  const colors = new Float32Array(starCount * 3);
  for (let i = 0; i < starCount; i += 1) {
    const u = random() * 2 - 1;
    const t = random() * Math.PI * 2;
    const s = Math.sqrt(1 - u * u);
    positions.set([Math.cos(t) * s * 70, u * 70, Math.sin(t) * s * 70], i * 3);
    const b = 0.25 + Math.pow(random(), 3) * 0.75;
    const warm = random() < 0.25;
    colors.set(warm ? [b, b * 0.86, b * 0.7] : [b * 0.88, b * 0.93, b], i * 3);
  }
  const starGeometry = track(new THREE.BufferGeometry());
  starGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  starGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  const starMaterial = track(
    new THREE.PointsMaterial({
      map: track(
        radialTexture(32, [
          [0, 'rgba(255, 255, 255, 1)'],
          [0.45, 'rgba(255, 255, 255, 0.7)'],
          [1, 'rgba(255, 255, 255, 0)'],
        ]),
      ),
      size: 2.4,
      sizeAttenuation: false,
      vertexColors: true,
      transparent: true,
      depthWrite: false,
    }),
  );
  scene.add(new THREE.Points(starGeometry, starMaterial));

  return { scene, bodies, starMaterial, disposables };
}

// Exponential approach of `value` to `goal` with rate k (1/s).
function approach(value, goal, k, dt) {
  return value + (goal - value) * (1 - Math.exp(-k * dt));
}

/**
 * Creates the orrery on a canvas: an HTMLCanvasElement on the main thread or
 * an OffscreenCanvas in a worker. It never touches the DOM. Input arrives as
 * messages through handle(); changes go out through emit():
 *
 *   { t: 'ready' }                       after the first frame
 *   { t: 'state', seq, hovered, keyed, followed, zoom, cursor }
 *   { t: 'anchor', x, y }                canvas px of the lit planet (or the
 *                                        centre) while a label is wanted
 *
 * Messages in: size, active, motion, label, pdown, pmove, pup, pcancel,
 * pleave, zoom, choose, follow, reset (see handle below). Nothing is drawn
 * while inactive.
 */
export function createOrrery(canvas, { width: w0, height: h0, pixelRatio, motion: motion0, emit }) {
  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: true,
    powerPreference: 'low-power',
  });
  renderer.setClearColor(0x000000, 0);
  renderer.setPixelRatio(pixelRatio);

  const { scene, bodies, starMaterial, disposables } = buildScene();
  starMaterial.size = 2.4 * pixelRatio;
  const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 200);
  const halfFovTan = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);
  const raf =
    typeof requestAnimationFrame === 'function'
      ? (cb) => requestAnimationFrame(cb)
      : (cb) => setTimeout(() => cb(performance.now()), 16);
  const caf =
    typeof cancelAnimationFrame === 'function' ? (id) => cancelAnimationFrame(id) : (id) => clearTimeout(id);

  let motion = motion0;
  let active = false;
  let width = 1;
  let height = 1;
  let frameDistance = 20;
  let wide = true;
  let offsetY = 0.5; // vertical position of the view centre, fraction of the height
  let simTime = 9; // a pleasing arrangement for the still frame

  // Camera: goal values set by input, current values ease towards them.
  const goal = { azimuth: 0.35, pitch: TILT, zoom: 1 };
  const cur = { azimuth: 0.35, pitch: TILT, zoom: 1 };
  const target = CENTRE.clone();
  const targetGoal = CENTRE.clone();
  let idleSpin = motion ? IDLE_SPIN : 0;
  let lastInput = -Infinity;

  // Selection state.
  let hovered = -1; // under the pointer
  let keyed = -1; // chosen from the keyboard
  let followed = -1; // camera follows it
  let seq = 0; // last command sequence number handled
  let cursor = '';
  let stateDirty = true;
  let labelWanted = false;
  let anchorX = NaN;
  let anchorY = NaN;

  // Pointer state.
  const pointer = { x: 0, y: 0, inside: false };
  let pickPending = false;
  let lastPick = 0;
  const presses = new Map(); // pointerId -> { x, y }
  let pressX = 0;
  let pressY = 0;
  let dragging = false;
  let pinchStart = 0;
  let pinchZoom = 1;

  const projected = new THREE.Vector3();

  function resize(w, h) {
    width = Math.max(1, w);
    height = Math.max(1, h);
    renderer.setSize(width, height, false);
    wide = width >= 900;
    // Where the orrery's centre sits in the canvas, and its outer radius in px.
    const radiusPx = wide ? Math.min(height * 0.48, width * 0.3) : Math.min(width * 0.47, height * 0.75);
    frameDistance = ((SCALE_RING + 0.3) * height) / (radiusPx * 2 * halfFovTan);
    camera.aspect = width / height;
    offsetY = offsetGoal();
    applyOffset();
  }

  // On narrow screens the hero band is short: a followed planet sits lower so
  // its card fits above it.
  function offsetGoal() {
    if (wide) return 0.5;
    return followed >= 0 ? 0.7 : 0.6;
  }

  function applyOffset() {
    const cx = wide ? 0.69 : 0.5;
    camera.setViewOffset(width, height, -(cx - 0.5) * width, -(offsetY - 0.5) * height, width, height);
    camera.updateProjectionMatrix();
  }

  function pose() {
    for (const b of bodies) {
      const angle = b.phase + b.omega * simTime;
      b.pivot.rotation.y = -angle;
      b.world.set(Math.cos(angle) * b.a, 0, Math.sin(angle) * b.a);
      if (b.moonPivot) b.moonPivot.rotation.y = -simTime * 1.4;
    }
  }

  function place() {
    const d = frameDistance * cur.zoom;
    const cp = Math.cos(cur.pitch);
    camera.position.set(
      target.x + Math.sin(cur.azimuth) * cp * d,
      target.y + Math.sin(cur.pitch) * d,
      target.z + Math.cos(cur.azimuth) * cp * d,
    );
    camera.lookAt(target);
    camera.updateMatrixWorld();
  }

  // Projects a world point into canvas pixels; false when behind the camera.
  function projectPoint(v) {
    projected.copy(v).project(camera);
    if (projected.z > 1) return false;
    projected.x = (projected.x + 1) * 0.5 * width;
    projected.y = (1 - projected.y) * 0.5 * height;
    return true;
  }

  // Nearest planet whose projected disc, plus some slop, contains (x, y).
  function pick(x, y) {
    let best = -1;
    let bestD2 = Infinity;
    for (let i = 0; i < bodies.length; i += 1) {
      if (!projectPoint(bodies[i].world)) continue;
      const dist = camera.position.distanceTo(bodies[i].world);
      const rpx = (bodies[i].r * height) / (2 * dist * halfFovTan);
      const dx = projected.x - x;
      const dy = projected.y - y;
      const d2 = dx * dx + dy * dy;
      const reach = rpx + PICK_SLOP;
      if (d2 <= reach * reach && d2 < bestD2) {
        best = i;
        bestD2 = d2;
      }
    }
    return best;
  }

  function highlighted() {
    if (keyed >= 0) return keyed;
    if (hovered >= 0) return hovered;
    return followed;
  }

  function sendState() {
    if (!stateDirty) return;
    stateDirty = false;
    emit({ t: 'state', seq, hovered, keyed, followed, zoom: goal.zoom, cursor });
  }

  function sendAnchor() {
    if (!labelWanted) return;
    const lit = highlighted();
    if (!projectPoint(lit >= 0 ? bodies[lit].world : CENTRE)) return;
    const x = Math.round(projected.x);
    const y = Math.round(projected.y);
    if (x === anchorX && y === anchorY) return;
    anchorX = x;
    anchorY = y;
    emit({ t: 'anchor', x, y });
  }

  let first = true;
  function draw() {
    pose();
    place();
    renderer.render(scene, camera);
    sendAnchor();
    if (first) {
      first = false;
      emit({ t: 'ready' });
    }
  }

  // ------------------------------------------------------------- frame loop

  let rafId = 0;
  let last = 0;
  let dirty = true;
  let compiled = false;

  function kick() {
    dirty = true;
    if (active && compiled && !rafId) {
      last = 0;
      rafId = raf(frame);
    }
  }

  function frame(now) {
    rafId = 0;
    if (!active) return;
    const dt = last ? Math.min((now - last) / 1000, 0.05) : 0;
    last = now;
    let settling = false;

    if (motion) {
      simTime += dt;
      const idle = !dragging && presses.size === 0 && hovered < 0 && now - lastInput > IDLE_DELAY;
      idleSpin = approach(idleSpin, idle ? IDLE_SPIN : 0, 1.2, dt);
      goal.azimuth += idleSpin * dt;
      cur.azimuth += idleSpin * dt;
    }

    // The followed planet moves, so the target goal is refreshed each frame.
    if (followed >= 0) {
      pose();
      targetGoal.copy(bodies[followed].world);
    }

    if (motion) {
      cur.azimuth = approach(cur.azimuth, goal.azimuth, dragging ? 30 : 6, dt);
      cur.pitch = approach(cur.pitch, goal.pitch, dragging ? 30 : 6, dt);
      cur.zoom = approach(cur.zoom, goal.zoom, 5, dt);
      target.x = approach(target.x, targetGoal.x, 4, dt);
      target.y = approach(target.y, targetGoal.y, 4, dt);
      target.z = approach(target.z, targetGoal.z, 4, dt);
    } else {
      cur.azimuth = goal.azimuth;
      cur.pitch = goal.pitch;
      cur.zoom = goal.zoom;
      target.copy(targetGoal);
    }
    const wantY = offsetGoal();
    if (offsetY !== wantY) {
      offsetY = motion ? approach(offsetY, wantY, 4, dt) : wantY;
      if (Math.abs(offsetY - wantY) < 1e-4) offsetY = wantY;
      applyOffset();
      settling = true;
    }
    if (
      Math.abs(cur.azimuth - goal.azimuth) > 1e-4 ||
      Math.abs(cur.pitch - goal.pitch) > 1e-4 ||
      Math.abs(cur.zoom - goal.zoom) > 1e-4 ||
      target.distanceToSquared(targetGoal) > 1e-8
    ) {
      settling = true;
    }

    // Highlight: halo, slightly larger body, brighter orbit.
    const lit = highlighted();
    for (let i = 0; i < bodies.length; i += 1) {
      const b = bodies[i];
      const want = i === lit ? 1 : 0;
      let next = motion ? approach(b.glow, want, 10, dt) : want;
      if (Math.abs(next - want) < 1e-3) next = want;
      if (next !== b.glow) {
        b.glow = next;
        settling = true;
      }
      b.planet.scale.setScalar(1 + 0.35 * b.glow);
      b.halo.visible = b.glow > 0.001;
      b.haloMat.opacity = 0.75 * b.glow;
      b.orbitMat.opacity = 0.24 + 0.36 * b.glow;
    }

    if (pickPending && now - lastPick >= PICK_INTERVAL) {
      pickPending = false;
      lastPick = now;
      pose();
      place();
      const hit = pointer.inside && !dragging ? pick(pointer.x, pointer.y) : -1;
      if (hit !== hovered) {
        hovered = hit;
        if (!dragging) setCursor(hit >= 0 ? 'pointer' : '');
        stateDirty = true;
        settling = true;
      }
    }

    if (motion || settling || dirty) {
      dirty = false;
      draw();
    }
    sendState();
    if (motion || settling || pickPending) rafId = raf(frame);
  }

  // ------------------------------------------------------------- actions

  function setCursor(value) {
    if (cursor !== value) {
      cursor = value;
      stateDirty = true;
    }
  }

  function follow(index) {
    followed = index;
    if (index >= 0) {
      pose();
      targetGoal.copy(bodies[index].world);
      goal.zoom = FOCUS_ZOOM;
    } else {
      targetGoal.copy(CENTRE);
      goal.zoom = 1;
      goal.pitch = TILT;
    }
    stateDirty = true;
  }

  function setZoom(z) {
    goal.zoom = THREE.MathUtils.clamp(z, ZOOM_MIN, ZOOM_MAX);
    stateDirty = true;
  }

  // ------------------------------------------------------------- pointer input

  function pointerMove(id, x, y) {
    pointer.x = x;
    pointer.y = y;
    pointer.inside = true;
    const press = presses.get(id);
    if (!press) {
      pickPending = true;
      return;
    }
    if (presses.size === 2) {
      press.x = x;
      press.y = y;
      let d = 0;
      let px = 0;
      let py = 0;
      let n = 0;
      for (const p of presses.values()) {
        if (n === 0) {
          px = p.x;
          py = p.y;
        } else {
          d = Math.hypot(p.x - px, p.y - py);
        }
        n += 1;
      }
      if (pinchStart > 0 && d > 0) {
        setZoom((pinchZoom * pinchStart) / d);
        cur.zoom = goal.zoom;
      }
      lastInput = performance.now();
      return;
    }
    const dx = x - press.x;
    const dy = y - press.y;
    if (!dragging && Math.hypot(x - pressX, y - pressY) > DRAG_THRESHOLD) {
      dragging = true;
      setCursor('grabbing');
    }
    if (dragging) {
      goal.azimuth -= dx * 0.006;
      goal.pitch = THREE.MathUtils.clamp(goal.pitch + dy * 0.005, PITCH_MIN, PITCH_MAX);
      lastInput = performance.now();
    }
    press.x = x;
    press.y = y;
  }

  function pointerDown(id, x, y) {
    presses.set(id, { x, y });
    if (presses.size === 1) {
      pressX = x;
      pressY = y;
      dragging = false;
    } else if (presses.size === 2) {
      let px = 0;
      let py = 0;
      let n = 0;
      for (const p of presses.values()) {
        if (n === 0) {
          px = p.x;
          py = p.y;
        } else {
          pinchStart = Math.hypot(p.x - px, p.y - py);
        }
        n += 1;
      }
      pinchZoom = goal.zoom;
      dragging = true;
    }
    lastInput = performance.now();
  }

  function pointerEnd(id, x, y, cancelled) {
    if (!presses.has(id)) return;
    presses.delete(id);
    if (presses.size > 0) return;
    pinchStart = 0;
    const wasDrag = dragging;
    dragging = false;
    setCursor(hovered >= 0 ? 'pointer' : '');
    lastInput = performance.now();
    if (cancelled || wasDrag) return;
    // A click or tap: follow the planet under it, or reset on empty sky.
    pose();
    place();
    const hit = pick(x, y);
    keyed = -1;
    stateDirty = true;
    if (hit >= 0) follow(hit === followed ? -1 : hit);
    else if (followed >= 0 || goal.zoom !== 1 || goal.pitch !== TILT) follow(-1);
  }

  // ------------------------------------------------------------- messages

  function handle(m) {
    if (m.seq) {
      seq = m.seq;
      stateDirty = true;
    }
    switch (m.t) {
      case 'size':
        resize(m.w, m.h);
        break;
      case 'active':
        active = m.on;
        if (!active && rafId) {
          caf(rafId);
          rafId = 0;
        }
        break;
      case 'motion':
        motion = m.on;
        if (!motion) idleSpin = 0;
        break;
      case 'label':
        labelWanted = m.on;
        anchorX = NaN;
        break;
      case 'pdown':
        pointerDown(m.id, m.x, m.y);
        break;
      case 'pmove':
        pointerMove(m.id, m.x, m.y);
        break;
      case 'pup':
        pointerEnd(m.id, m.x, m.y, false);
        break;
      case 'pcancel':
        pointerEnd(m.id, 0, 0, true);
        break;
      case 'pleave':
        pointer.inside = false;
        pickPending = true;
        break;
      case 'zoom':
        setZoom(m.z);
        lastInput = performance.now();
        break;
      case 'choose':
        keyed = m.i;
        stateDirty = true;
        lastInput = performance.now();
        break;
      case 'follow':
        keyed = -1;
        follow(m.i);
        lastInput = performance.now();
        break;
      case 'reset':
        keyed = -1;
        follow(-1);
        lastInput = performance.now();
        break;
      default:
        return;
    }
    kick();
  }

  resize(w0, h0);
  pose();
  place();
  // Shaders compile in the background where the driver allows it.
  renderer
    .compileAsync(scene, camera)
    .catch(() => {})
    .then(() => {
      compiled = true;
      kick();
    });

  function dispose() {
    active = false;
    if (rafId) caf(rafId);
    for (const d of disposables) d.dispose();
    renderer.dispose();
  }

  return { handle, dispose };
}
