# ADR-017: Half-resolution geodesic trace with full-resolution sky lookup

## Context
ADR-003 fixes the physics of black-hole rendering: Schwarzschild null
geodesics in u = 1/r, integrated with RK4 under a capped step count, and a
thin disk with Doppler beaming and gravitational redshift. The spec (7.3)
asks for a half-resolution compute pass, at most 512 steps per ray and at
least 30 fps at 1080p on a GTX 1660-class GPU. Two decisions follow that
ADR-003 leaves open: what the half-resolution pass stores and how it reaches
full resolution, and what the step budget means.

A half-resolution image of the lensed sky blurs every star into a 2 x 2 pixel
smear and doubles the step of the shadow edge, while the disk light varies
smoothly almost everywhere. The map from pixel to escape direction is smooth
except across the critical curve b = 3 sqrt(3) M, which is known analytically
for every pixel.

## Decision
- The compute pass traces one ray per half-resolution sample (samples at the
  pixel corners (2i + 1, 2j + 1), so every pixel centre has four samples
  around it) and stores the geodesic field, not a colour: the disk light that
  reaches the camera, the sky transmittance, the escape direction and the
  depth of the first opaque hit.
- A full-resolution composite pass interpolates that field with bilinear
  weights, leaves out samples on the other side of the analytic critical
  curve, interpolates escape directions (nearest sample where they jump), and
  looks the sky up once per pixel in an environment cube map rendered from the
  existing starfield and Milky Way, with the mip level from the angular
  footprint of the lensed direction. The shadow edge is anti-aliased from the
  analytic impact parameter.
- Rays in a band of 8 percent around the critical impact parameter, where
  the higher-order disk images are thinner than a sample, are supersampled
  with the four pixel centres of their sample.
- The step budget N (64 to 512, default 512) sets the base step 4 pi / N in
  the orbit angle: the budget covers two full turns of the photon plane,
  steps are shortened only for steep (nearly radial) rays, and a ray that
  exhausts it has circled the photon sphere and is counted as captured. A
  smaller budget therefore trades accuracy for time without cutting the
  image off.
- A double-precision CPU reference in orrery_core (graphics-free) implements
  the same integrator, step rule and disk model; tests compare it with the
  shader ray by ray through a geometry output mode of the compute pass.

## Consequences
Stars keep their full-resolution sharpness, the shadow edge is exact and
smooth, and the geodesics cost a quarter of a full-resolution trace (measured
about 1 ms per 1080p frame on the reference GPU, far inside the 33 ms
budget). The environment map must be rebuilt when the sky brightness
settings change (a few tens of milliseconds) and holds 134 MB (2048^2 faces,
R11G11B10F, with mipmaps). Bodies are not lensed: they are drawn in the
normal pass, hidden by the shadow and the disk through the depth the
composite writes. Sub-sample disk features off the critical band (a disk
smaller than a few pixels) are undersampled; an unresolved hole keeps its
marker.

Alternatives rejected: a full-resolution trace (four times the cost for
detail only the sky and the shadow edge need, both obtained exactly by the
composite), upsampling a half-resolution colour image (smears stars and the
shadow edge), a fixed angular step that truncates long rays (cuts the image
near the photon ring when the budget is lowered), and lensing the bodies
(each body would need its own lensed images; out of the V1 scope).

## Status
Accepted
