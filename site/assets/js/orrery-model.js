// The hero orrery's model: circular, coplanar orbits whose angular speeds
// follow Kepler's third law (omega proportional to a^-3/2). Kept free of
// three.js so the page can describe the planets without loading the renderer.

// Orbit radius, planet radius, colour, starting phase (radians).
export const PLANETS = [
  { a: 1.05, r: 0.07, color: 0xb9b2a7, phase: 0.6 },
  { a: 1.65, r: 0.11, color: 0xe0c08f, phase: 2.5 },
  { a: 2.35, r: 0.12, color: 0x74aacc, phase: 4.3, moon: true },
  { a: 3.15, r: 0.09, color: 0xd9825f, phase: 1.2 },
  { a: 4.05, r: 0.21, color: 0xd8b98a, phase: 3.4 },
  { a: 5.0, r: 0.17, color: 0xc9a46a, phase: 5.5, ring: true },
];

export const BASE_OMEGA = 0.3; // rad/s of the innermost planet

// Camera zoom limits as fractions of the framing distance; the page needs them
// to decide whether a wheel event zooms or scrolls.
export const ZOOM_MIN = 0.3;
export const ZOOM_MAX = 1.35;

/** Angular speed of planet i in rad/s. */
export function omega(i) {
  return BASE_OMEGA * Math.pow(PLANETS[i].a / PLANETS[0].a, -1.5);
}

/** What the labels say about each planet; all of it follows from the table above. */
export const PLANET_FACTS = PLANETS.map((p, i) => ({
  radius: p.a / PLANETS[0].a, // orbit radius relative to the innermost
  period: Math.pow(p.a / PLANETS[0].a, 1.5), // period relative to the innermost
  seconds: (2 * Math.PI) / omega(i), // one turn on screen, at full speed
  moon: Boolean(p.moon),
  ring: Boolean(p.ring),
}));
