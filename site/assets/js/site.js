// Page behaviour: copy buttons, scroll reveals and the hero orrery loader.
// Everything here is progressive: without JavaScript the page is complete.

const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');

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
if (!reducedMotion.matches && 'IntersectionObserver' in window) {
  document.documentElement.classList.add('reveal-on');
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

function webglAvailable() {
  try {
    const probe = document.createElement('canvas');
    return Boolean(probe.getContext('webgl2') || probe.getContext('webgl'));
  } catch {
    return false;
  }
}

const hero = document.querySelector('[data-orrery]');
const canvas = hero && hero.querySelector('canvas');

if (canvas && webglAvailable()) {
  let scene = null;
  let loading = false;
  let inView = false;

  const update = () => {
    if (!scene) return;
    if (inView && document.visibilityState === 'visible') scene.start();
    else scene.stop();
  };

  const load = async () => {
    if (loading) return;
    loading = true;
    try {
      const { mountOrrery } = await import(new URL('./orrery-scene.js', import.meta.url).href);
      scene = mountOrrery(canvas, {
        still: reducedMotion.matches,
        onFirstFrame: () => hero.classList.add('is-live'),
      });
      canvas.addEventListener('webglcontextlost', () => {
        hero.classList.remove('is-live');
        scene.stop();
      });
      update();
    } catch (error) {
      // The static drawing stays in place.
      hero.classList.remove('is-live');
      console.warn('Orrery motif unavailable:', error);
    }
  };

  new IntersectionObserver((entries) => {
    inView = entries[entries.length - 1].isIntersecting;
    if (inView) load();
    update();
  }).observe(canvas);
  document.addEventListener('visibilitychange', update);
}
