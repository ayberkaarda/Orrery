// Logarithmic depth snippet. Composed after the #version line of a stage
// (see orrery::render::compose_shader) and never compiled on its own.
//
// depth = log2(1 + w) / log2(1 + far), with w the view-space distance along
// the view axis in metres (clip-space w). Matches orrery::render::log_depth()
// on the CPU. Written to gl_FragDepth per fragment, so it stays exact for
// large triangles that cross many orders of magnitude in w.

uniform float u_log_depth_scale; // 1 / log2(1 + far)

float orrery_log_depth(float view_w) {
    return log2(max(1.0 + view_w, 1.0e-6)) * u_log_depth_scale;
}
