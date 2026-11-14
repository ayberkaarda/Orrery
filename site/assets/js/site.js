// Page behaviour: copy buttons, scroll reveals and the hero orrery loader. Everything here is progressive: without JavaScript the
// page is complete and still.

const root = document.documentElement;
const lang = root.lang === 'tr' ? 'tr' : 'en';
const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');

const STRINGS = {
  en: {
    planet: (n) => `Planet ${n}`,
    innermost: 'Innermost orbit: the reference period',
    period: (t) => `Period ${t}× the innermost`,
    orbit: (a) => `Orbit radius ${a}× the innermost`,
    turn: (s) => `One turn every ${s} s here`,
    moon: 'Carries a moon',
    ring: 'Ringed',
    hint: 'Arrow keys choose a planet · Enter follows it · Esc resets',
    hintTitle: 'Orrery',
    canvas:
      'Interactive orrery. Arrow keys or Tab choose a planet, Enter follows it with the camera, Escape resets the view, plus and minus zoom.',
    chosen: (name, fact) => `${name}. ${fact}. Press Enter to follow it.`,
    following: (name, fact) => `Following ${name}. ${fact}.`,
    reset: 'View reset.',
  },
  tr: {
    planet: (n) => `Gezegen ${n}`,
    innermost: 'En iç yörünge: periyotlar buna göre',
    period: (t) => `Periyodu en içtekinin ${t} katı`,
    orbit: (a) => `Yörünge yarıçapı en içtekinin ${a} katı`,
    turn: (s) => `Burada bir tur ${s} sn`,
    moon: 'Bir uydusu var',
    ring: 'Halkalı',
    hint: 'Ok tuşları gezegen seçer · Enter izler · Esc sıfırlar',
    hintTitle: 'Orrery',
    canvas:
      'Etkileşimli orrery. Ok tuşları veya Tab ile bir gezegen seçin; Enter kamerayla onu izler, Escape görünümü sıfırlar, artı ve eksi yakınlaştırır.',
    chosen: (name, fact) => `${name}. ${fact}. İzlemek için Enter'a basın.`,
    following: (name, fact) => `${name} izleniyor. ${fact}.`,
    reset: 'Görünüm sıfırlandı.',
  },
}[lang];

// ---------------------------------------------------------------- motion
//
// Motion follows the system's reduced-motion setting.

const motionListeners = [];
const motionOn = () => !reducedMotion.matches;
reducedMotion.addEventListener('change', () => {
  for (const listener of motionListeners) listener(motionOn());
});

// ---------------------------------------------------------------- copy buttons

async function copyText(text) {
  if (navigator.clipboard && window.isSecureContext) {
    try {
      await navigator.clipboard.writeText(text);
      return;
    } catch {
      // Permission denied or no focus: fall back to a selection copy.
    }
  }
  const area = document.createElement('textarea');
  area.value = text;
  area.setAttribute('readonly', '');
  area.style.position = 'fixed';
  area.style.opacity = '0';
  document.body.appendChild(area);
  area.select();
  const ok = document.execCommand('copy');
  area.remove();
  if (!ok) throw new Error('copy failed');
}

const status = document.getElementById('copy-status');

for (const button of document.querySelectorAll('[data-copy]')) {
  button.hidden = false;
  const label = button.querySelector('.copy__label');
  const idle = label ? label.textContent : '';
  button.addEventListener('click', async () => {
    const target = document.getElementById(button.dataset.copy);
    if (!target) return;
    let message;
    try {
      await copyText(target.textContent.trim());
      message = button.dataset.copied || 'Copied';
      button.classList.add('is-copied');
    } catch {
      message = button.dataset.failed || 'Copy failed';
    }
    if (label) label.textContent = message;
    if (status) status.textContent = message;
    window.setTimeout(() => {
      if (label) label.textContent = idle;
      button.classList.remove('is-copied');
    }, 1800);
  });
}

// ---------------------------------------------------------------- scroll reveals

const revealables = document.querySelectorAll('[data-reveal]');
if (motionOn() && 'IntersectionObserver' in window) {
  root.classList.add('reveal-on');
  const io = new IntersectionObserver(
    (entries) => {
      for (const entry of entries) {
        if (entry.isIntersecting) {
          entry.target.classList.add('is-visible');
          io.unobserve(entry.target);
        }
      }
    },
    { rootMargin: '0px 0px -8% 0px', threshold: 0.08 },
  );
  for (const el of revealables) io.observe(el);
}

// ---------------------------------------------------------------- hero orrery
//
// The scene itself (orrery-scene.js) never touches the DOM. Where the browser
// can, it runs in a worker on an OffscreenCanvas; otherwise on this thread.
// Either way this side turns DOM events into messages, mirrors the scene's
// state, and owns the label and the screen-reader announcements.

const hero = document.querySelector('[data-orrery]');
const sky = hero && hero.querySelector('.hero__sky');

function planetText(index, facts) {
  const fmt = new Intl.NumberFormat(lang, { maximumFractionDigits: 2, minimumFractionDigits: 2 });
  const whole = new Intl.NumberFormat(lang, { maximumFractionDigits: 0 });
  const f = facts[index];
  const name = STRINGS.planet(['I', 'II', 'III', 'IV', 'V', 'VI', 'VII', 'VIII'][index]);
  const fact = index === 0 ? STRINGS.innermost : STRINGS.period(fmt.format(f.period));
  const more = [];
  if (index > 0) more.push(STRINGS.orbit(fmt.format(f.radius)));
  more.push(STRINGS.turn(whole.format(f.seconds)));
  if (f.moon) more.push(STRINGS.moon);
  if (f.ring) more.push(STRINGS.ring);
  return { name, fact, more };
}

function buildLabel() {
  const label = document.createElement('div');
  label.className = 'orrery-label';
  label.setAttribute('aria-hidden', 'true');
  label.hidden = true;
  const name = document.createElement('p');
  name.className = 'orrery-label__name';
  const fact = document.createElement('p');
  fact.className = 'orrery-label__fact';
  const more = document.createElement('p');
  more.className = 'orrery-label__more';
  label.append(name, fact, more);
  return { label, name, fact, more };
}

// Runs fn once the page has loaded, its first contentful paint is out and the
// main thread is idle, so the 3D scene never holds back the first paint.
function afterFirstPaint(fn) {
  const loaded = new Promise((resolve) => {
    if (document.readyState === 'complete') resolve();
    else window.addEventListener('load', resolve, { once: true });
  });
  const painted = new Promise((resolve) => {
    window.setTimeout(resolve, 4000); // in case paint timing is unavailable
    try {
      const observer = new PerformanceObserver((list) => {
        if (list.getEntriesByName('first-contentful-paint').length) {
          observer.disconnect();
          resolve();
        }
      });
      observer.observe({ type: 'paint', buffered: true });
    } catch {
      // No paint timing: a frame callback followed by a timeout runs after a paint.
      window.requestAnimationFrame(() => window.setTimeout(resolve, 0));
    }
  });
  Promise.all([loaded, painted]).then(() => {
    if ('requestIdleCallback' in window) window.requestIdleCallback(fn, { timeout: 2000 });
    else window.setTimeout(fn, 200);
  });
}

// Starts the scene in a worker; resolves to a port, or null if that failed
// before the canvas was handed over.
function startInWorker(canvas, init, onMessage) {
  if (!('transferControlToOffscreen' in canvas) || typeof Worker === 'undefined') return null;
  let worker;
  try {
    worker = new Worker(new URL('./orrery-worker.js', import.meta.url), { type: 'module' });
  } catch {
    return null;
  }
  const offscreen = canvas.transferControlToOffscreen();
  worker.addEventListener('message', (event) => onMessage(event.data));
  worker.addEventListener('error', () => onMessage({ t: 'error', message: 'worker failed' }));
  worker.postMessage({ t: 'init', canvas: offscreen, ...init }, [offscreen]);
  return {
    post: (m) => worker.postMessage(m),
    stop: () => worker.terminate(),
  };
}

async function startHere(canvas, init, onMessage) {
  const { createOrrery } = await import(new URL('./orrery-scene.js', import.meta.url).href);
  const orrery = createOrrery(canvas, { ...init, emit: onMessage });
  canvas.addEventListener('webglcontextlost', () => onMessage({ t: 'lost' }));
  return {
    post: (m) => orrery.handle(m),
    stop: () => orrery.dispose(),
  };
}

if (sky) {
  const live = hero.querySelector('[data-orrery-live]');
  let canvas = sky.querySelector('canvas');
  let port = null;
  let started = false;
  let inView = false;
  let ready = false;
  let triedHere = false;

  // Mirror of the scene's state. Commands bump seq; a state message older
  // than the last command only updates what the scene alone decides.
  const mirror = { hovered: -1, keyed: -1, followed: -1, zoom: 1, cursor: '' };
  let seq = 0;
  let facts = null;
  let zoomMin = 0.3;
  let zoomMax = 1.35;
  let parts = null;
  let width = 1;
  let height = 1;
  let shownIndex = -2;
  let shownMode = '';
  let anchor = null;

  const post = (m) => {
    if (port) port.post(m);
  };

  const announce = (text) => {
    if (!live) return;
    // Clearing first makes a repeated message audible again.
    live.textContent = '';
    window.setTimeout(() => {
      live.textContent = text;
    }, 60);
  };

  const keyboardFocus = () => document.activeElement === canvas && canvas.matches(':focus-visible');

  function placeLabel() {
    if (!anchor || !shownMode) return;
    // Above and to the right of the body; flipped left or below when there is
    // no room, then kept inside the canvas and clear of the page header.
    const { label } = parts;
    const w = label.offsetWidth;
    const h = label.offsetHeight;
    const top = 76;
    let x = anchor.x + 18;
    let y = anchor.y - h - 12;
    if (x + w > width - 12) x = anchor.x - w - 18;
    if (y < top) y = anchor.y + 18;
    x = Math.round(Math.min(Math.max(x, 12), Math.max(12, width - w - 12)));
    y = Math.round(Math.min(Math.max(y, top), Math.max(top, height - h - 12)));
    label.style.transform = `translate3d(${x}px, ${y}px, 0)`;
    label.style.visibility = '';
  }

  function refreshLabel() {
    if (!parts) return;
    const lit = mirror.keyed >= 0 ? mirror.keyed : mirror.hovered >= 0 ? mirror.hovered : mirror.followed;
    let mode = '';
    if (lit >= 0) mode = lit === mirror.followed ? 'card' : 'hover';
    else if (keyboardFocus()) mode = 'hint';
    if (lit === shownIndex && mode === shownMode) return;
    const wasShown = Boolean(shownMode);
    shownIndex = lit;
    shownMode = mode;
    const { label, name, fact, more } = parts;
    if (mode === 'hint') {
      name.textContent = STRINGS.hintTitle;
      fact.textContent = STRINGS.hint;
      more.textContent = '';
    } else if (mode) {
      const text = planetText(lit, facts);
      name.textContent = text.name;
      fact.textContent = text.fact;
      more.textContent = mode === 'card' ? text.more.join('\n') : '';
    }
    label.hidden = !mode;
    label.dataset.mode = mode;
    if (mode && !wasShown) {
      // Hidden until the scene reports where the body is.
      anchor = null;
      label.style.visibility = 'hidden';
      post({ t: 'label', on: true });
    } else if (!mode && wasShown) {
      post({ t: 'label', on: false });
    } else {
      placeLabel();
    }
  }

  function setFollowed(index) {
    if (index === mirror.followed) return;
    mirror.followed = index;
    if (index < 0) {
      announce(STRINGS.reset);
    } else {
      const text = planetText(index, facts);
      announce(STRINGS.following(text.name, [text.fact, ...text.more].join('. ')));
    }
  }

  function onMessage(m) {
    switch (m.t) {
      case 'ready':
        ready = true;
        hero.classList.add('is-live');
        canvas.tabIndex = 0;
        canvas.setAttribute('role', 'application');
        canvas.setAttribute('aria-label', STRINGS.canvas);
        break;
      case 'state':
        mirror.hovered = m.hovered;
        mirror.cursor = m.cursor;
        canvas.style.cursor = m.cursor;
        if (m.seq >= seq) {
          mirror.keyed = m.keyed;
          mirror.zoom = m.zoom;
          setFollowed(m.followed);
        }
        refreshLabel();
        break;
      case 'anchor':
        anchor = m;
        placeLabel();
        break;
      case 'lost':
      case 'error':
        fail(m.message);
        break;
      default:
        break;
    }
  }

  // ------------------------------------------------------------- commands

  function choose(index) {
    mirror.keyed = index;
    seq += 1;
    post({ t: 'choose', i: index, seq });
    if (index >= 0) {
      const text = planetText(index, facts);
      announce(STRINGS.chosen(text.name, text.fact));
    }
    refreshLabel();
  }

  function follow(index) {
    mirror.keyed = -1;
    seq += 1;
    post({ t: index >= 0 ? 'follow' : 'reset', i: index, seq });
    setFollowed(index);
    if (index < 0) mirror.zoom = 1;
    refreshLabel();
  }

  function zoomTo(z) {
    mirror.zoom = z;
    seq += 1;
    post({ t: 'zoom', z, seq });
  }

  // ------------------------------------------------------------- DOM events

  const local = (event) => {
    const rect = canvas.getBoundingClientRect();
    return [event.clientX - rect.left, event.clientY - rect.top];
  };

  const onPointerMove = (event) => {
    const [x, y] = local(event);
    post({ t: 'pmove', id: event.pointerId, x, y });
  };
  const onPointerDown = (event) => {
    if (event.pointerType === 'mouse' && event.button !== 0) return;
    const [x, y] = local(event);
    try {
      canvas.setPointerCapture(event.pointerId);
    } catch {
      // Capture is a convenience; dragging still works inside the canvas.
    }
    post({ t: 'pdown', id: event.pointerId, x, y });
  };
  const onPointerUp = (event) => {
    const [x, y] = local(event);
    post({ t: 'pup', id: event.pointerId, x, y });
  };
  const onPointerCancel = (event) => post({ t: 'pcancel', id: event.pointerId });
  const onPointerLeave = () => post({ t: 'pleave' });

  // The wheel zooms only once the canvas has focus (it was clicked or tabbed
  // to); otherwise, and at either zoom limit, the page scrolls as usual.
  const onWheel = (event) => {
    if (document.activeElement !== canvas || event.ctrlKey) return;
    const scale = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? height : 1;
    const delta = event.deltaY * scale;
    if (delta === 0) return;
    const next = Math.min(zoomMax, Math.max(zoomMin, mirror.zoom * Math.exp(delta * 0.0015)));
    if (Math.abs(next - mirror.zoom) < 1e-6) return;
    event.preventDefault();
    zoomTo(next);
  };

  const onKeyDown = (event) => {
    if (event.altKey || event.ctrlKey || event.metaKey) return;
    const n = facts.length;
    const at = mirror.keyed >= 0 ? mirror.keyed : mirror.followed;
    switch (event.key) {
      case 'ArrowRight':
      case 'ArrowDown':
        event.preventDefault();
        choose(at < 0 ? 0 : (at + 1) % n);
        break;
      case 'ArrowLeft':
      case 'ArrowUp':
        event.preventDefault();
        choose(at < 0 ? n - 1 : (at - 1 + n) % n);
        break;
      case 'Home':
        event.preventDefault();
        choose(0);
        break;
      case 'End':
        event.preventDefault();
        choose(n - 1);
        break;
      case 'Tab':
        // Steps through the planets, then lets focus leave the canvas.
        if (event.shiftKey ? at > 0 : at < n - 1) {
          event.preventDefault();
          choose(event.shiftKey ? at - 1 : at + 1);
        }
        break;
      case 'Enter':
      case ' ':
        if (at >= 0) {
          event.preventDefault();
          follow(at);
        }
        break;
      case 'Escape':
        if (mirror.keyed >= 0 || mirror.followed >= 0 || mirror.zoom !== 1) {
          event.preventDefault();
          follow(-1);
        }
        break;
      case '+':
      case '=':
        event.preventDefault();
        zoomTo(Math.max(zoomMin, mirror.zoom / 1.2));
        break;
      case '-':
      case '_':
        event.preventDefault();
        zoomTo(Math.min(zoomMax, mirror.zoom * 1.2));
        break;
      default:
        break;
    }
  };

  const onFocus = () => refreshLabel();
  const onBlur = () => {
    if (mirror.keyed >= 0) {
      mirror.keyed = -1;
      seq += 1;
      post({ t: 'choose', i: -1, seq });
    }
    refreshLabel();
  };

  function bind(target) {
    target.addEventListener('pointermove', onPointerMove, { passive: true });
    target.addEventListener('pointerdown', onPointerDown);
    target.addEventListener('pointerup', onPointerUp);
    target.addEventListener('pointercancel', onPointerCancel);
    target.addEventListener('pointerleave', onPointerLeave, { passive: true });
    target.addEventListener('wheel', onWheel, { passive: false });
    target.addEventListener('keydown', onKeyDown);
    target.addEventListener('focus', onFocus);
    target.addEventListener('blur', onBlur);
  }

  // ------------------------------------------------------------- lifecycle

  const updateActive = () => post({ t: 'active', on: inView && document.visibilityState === 'visible' });

  function init() {
    width = Math.max(1, canvas.clientWidth);
    height = Math.max(1, canvas.clientHeight);
    return {
      width,
      height,
      pixelRatio: Math.min(window.devicePixelRatio || 1, 1.75),
      motion: motionOn(),
    };
  }

  // A canvas handed to a worker cannot be drawn on here, so a fallback after a
  // worker failure gets a fresh one.
  async function fail(message) {
    hero.classList.remove('is-live');
    canvas.removeAttribute('tabindex');
    canvas.removeAttribute('role');
    canvas.removeAttribute('aria-label');
    if (port) port.stop();
    port = null;
    if (ready || triedHere) {
      if (message) console.warn('Orrery motif unavailable:', message);
      return;
    }
    triedHere = true;
    const fresh = canvas.cloneNode(false);
    canvas.replaceWith(fresh);
    canvas = fresh;
    bind(canvas);
    try {
      port = await startHere(canvas, init(), onMessage);
      post({ t: 'size', w: width, h: height });
      updateActive();
    } catch (error) {
      // No WebGL at all: the static drawing stays in place.
      console.warn('Orrery motif unavailable:', error);
    }
  }

  async function start() {
    if (started) return;
    started = true;
    try {
      const model = await import(new URL('./orrery-model.js', import.meta.url).href);
      facts = model.PLANET_FACTS;
      zoomMin = model.ZOOM_MIN;
      zoomMax = model.ZOOM_MAX;
      parts = buildLabel();
      hero.append(parts.label);
      bind(canvas);
      port = startInWorker(canvas, init(), onMessage);
      if (!port) {
        triedHere = true;
        port = await startHere(canvas, init(), onMessage);
      }
      motionListeners.push((on) => post({ t: 'motion', on }));
      new ResizeObserver(() => {
        width = Math.max(1, canvas.clientWidth);
        height = Math.max(1, canvas.clientHeight);
        post({ t: 'size', w: width, h: height });
        placeLabel();
      }).observe(sky);
      updateActive();
    } catch (error) {
      hero.classList.remove('is-live');
      console.warn('Orrery motif unavailable:', error);
    }
  }

  let afterPaint = false;
  afterFirstPaint(() => {
    afterPaint = true;
    if (inView) start();
  });

  new IntersectionObserver((entries) => {
    inView = entries[entries.length - 1].isIntersecting;
    if (inView && afterPaint) start();
    updateActive();
  }).observe(sky);
  document.addEventListener('visibilitychange', updateActive);
}
