// Hero motif: a mechanical orrery drawn with three.js.
//
// Circular, coplanar orbits whose angular speeds follow Kepler's third law
// (omega proportional to a^-3/2). Each planet rides on a brass arm from the
// central column, as on a mechanical orrery. Decorative only: the screenshots
// further down the page come from the application itself.

import * as THREE from '../vendor/three/three.module.min.js';

const BRASS = 0xe2a75c;
const STEEL = 0x74aacc;
const ORBIT = 0x8e96a4;

// Orbit radius, planet radius, colour, starting phase (radians).
const PLANETS = [
  { a: 1.05, r: 0.07, color: 0xb9b2a7, phase: 0.6 },
  { a: 1.65, r: 0.11, color: 0xe0c08f, phase: 2.5 },
  { a: 2.35, r: 0.12, color: 0x74aacc, phase: 4.3, moon: true },
  { a: 3.15, r: 0.09, color: 0xd9825f, phase: 1.2 },
  { a: 4.05, r: 0.21, color: 0xd8b98a, phase: 3.4 },
  { a: 5.0, r: 0.17, color: 0xc9a46a, phase: 5.5, ring: true },
];
const SCALE_RING = 5.75;
const BASE_OMEGA = 0.3; // rad/s of the innermost planet
const TILT = THREE.MathUtils.degToRad(23); // camera elevation above the plane

function glowTexture() {
  const size = 128;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const g = canvas.getContext('2d');
  const grad = g.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  grad.addColorStop(0, 'rgba(255, 236, 205, 1)');
  grad.addColorStop(0.18, 'rgba(255, 206, 140, 0.55)');
  grad.addColorStop(0.45, 'rgba(226, 167, 92, 0.14)');
  grad.addColorStop(1, 'rgba(226, 167, 92, 0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function dotTexture() {
  const size = 32;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const g = canvas.getContext('2d');
  const grad = g.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  grad.addColorStop(0, 'rgba(255, 255, 255, 1)');
  grad.addColorStop(0.45, 'rgba(255, 255, 255, 0.7)');
  grad.addColorStop(1, 'rgba(255, 255, 255, 0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, size, size);
  return new THREE.CanvasTexture(canvas);
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
  const glowMap = track(glowTexture());
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
  const orbitMat = track(lineMaterial(ORBIT, 0.24));
  const armMat = track(lineMaterial(BRASS, 0.55));
  const bodies = PLANETS.map((p, i) => {
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
      track(new THREE.MeshStandardMaterial({ color: p.color, roughness: 0.8, metalness: 0.05 })),
    );
    planet.position.set(p.a, 0, 0);
    pivot.add(planet);

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
        track(new THREE.MeshStandardMaterial({ color: 0xd4d6da, roughness: 0.9 })),
      );
      moon.position.set(0.3, 0, 0);
      moonPivot.add(moon);
    }

    return {
      pivot,
      moonPivot,
      phase: p.phase,
      omega: BASE_OMEGA * Math.pow(p.a / PLANETS[0].a, -1.5),
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
      map: track(dotTexture()),
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

/**
 * Mounts the orrery on a canvas.
 * Returns { start, stop, dispose }. With `still: true` it draws one frame and
 * never animates (reduced motion).
 */
export function mountOrrery(canvas, { still = false, onFirstFrame = () => {} } = {}) {
  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: true,
    powerPreference: 'low-power',
  });
  renderer.setClearColor(0x000000, 0);
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 1.75);
  renderer.setPixelRatio(pixelRatio);

  const { scene, bodies, starMaterial, disposables } = buildScene();
  starMaterial.size = 2.4 * pixelRatio;
  const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 200);
  const fovFactor = 2 * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);

  let distance = 20;
  let width = 1;
  let height = 1;
  let simTime = still ? 9 : 0;
  let azimuth = 0.35;
  const pointer = { x: 0, y: 0 };
  const eased = { x: 0, y: 0 };

  function layout() {
    width = Math.max(1, canvas.clientWidth);
    height = Math.max(1, canvas.clientHeight);
    renderer.setSize(width, height, false);
    const wide = width >= 900;
    // Where the orrery's centre sits in the canvas, and its outer radius in px.
    const cx = wide ? 0.69 : 0.5;
    const cy = wide ? 0.5 : 0.6;
    const radiusPx = wide ? Math.min(height * 0.48, width * 0.3) : Math.min(width * 0.47, height * 0.75);
    distance = ((SCALE_RING + 0.3) * height) / (radiusPx * fovFactor);
    camera.aspect = width / height;
    camera.setViewOffset(width, height, -(cx - 0.5) * width, -(cy - 0.5) * height, width, height);
    camera.updateProjectionMatrix();
  }

  function place() {
    const yaw = azimuth + eased.x * 0.08;
    const pitch = TILT + eased.y * 0.04;
    camera.position.set(
      Math.sin(yaw) * Math.cos(pitch) * distance,
      Math.sin(pitch) * distance,
      Math.cos(yaw) * Math.cos(pitch) * distance,
    );
    camera.lookAt(0, -0.35, 0);
  }

  function pose() {
    for (const b of bodies) {
      b.pivot.rotation.y = -(b.phase + b.omega * simTime);
      if (b.moonPivot) b.moonPivot.rotation.y = -simTime * 1.4;
    }
  }

  let first = true;
  let running = false;
  function draw() {
    pose();
    place();
    renderer.render(scene, camera);
    if (first) {
      first = false;
      onFirstFrame();
    }
  }

  let last = 0;
  function frame(now) {
    const dt = last ? Math.min((now - last) / 1000, 0.05) : 0;
    last = now;
    simTime += dt;
    azimuth += dt * 0.012;
    eased.x += (pointer.x - eased.x) * Math.min(1, dt * 2.5);
    eased.y += (pointer.y - eased.y) * Math.min(1, dt * 2.5);
    draw();
  }

  function onPointer(event) {
    pointer.x = (event.clientX / window.innerWidth) * 2 - 1;
    pointer.y = (event.clientY / window.innerHeight) * 2 - 1;
  }

  const resizeObserver = new ResizeObserver(() => {
    layout();
    if (still || !running) draw();
  });
  resizeObserver.observe(canvas);
  layout();

  function start() {
    if (still || running) return;
    running = true;
    last = 0;
    renderer.setAnimationLoop(frame);
  }
  function stop() {
    if (!running) return;
    running = false;
    renderer.setAnimationLoop(null);
  }

  if (still) {
    draw();
  } else {
    window.addEventListener('pointermove', onPointer, { passive: true });
  }

  function dispose() {
    stop();
    resizeObserver.disconnect();
    window.removeEventListener('pointermove', onPointer);
    for (const d of disposables) d.dispose();
    renderer.dispose();
  }

  return { start, stop, dispose, draw };
}
