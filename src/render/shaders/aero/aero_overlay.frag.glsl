#version 460 core
// Overlay texture: premultiplied sRGB-encoded values, blended with ONE,
// ONE_MINUS_SRC_ALPHA into the sRGB target with GL_FRAMEBUFFER_SRGB off (the
// stored values are blended directly, as the user interface does).

in vec2 v_uv;

layout(binding = 0) uniform sampler2D u_image;

layout(location = 0) out vec4 o_color;

void main() {
    o_color = texelFetch(u_image, ivec2(v_uv * vec2(textureSize(u_image, 0))), 0);
}
