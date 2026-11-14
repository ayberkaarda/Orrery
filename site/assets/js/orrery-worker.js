// Runs the hero orrery off the main thread. The page transfers its canvas
// here (OffscreenCanvas) and forwards input as messages, so creating the WebGL
// context, compiling shaders and drawing frames never block the page.

import { createOrrery } from './orrery-scene.js';

let orrery = null;

self.addEventListener('message', (event) => {
  const m = event.data;
  if (m.t !== 'init') {
    if (orrery) orrery.handle(m);
    return;
  }
  try {
    m.canvas.addEventListener('webglcontextlost', () => self.postMessage({ t: 'lost' }));
    orrery = createOrrery(m.canvas, {
      width: m.width,
      height: m.height,
      pixelRatio: m.pixelRatio,
      motion: m.motion,
      emit: (message) => self.postMessage(message),
    });
  } catch (error) {
    self.postMessage({ t: 'error', message: String((error && error.message) || error) });
  }
});
