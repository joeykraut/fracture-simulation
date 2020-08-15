#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform sampler2D u_texture_1;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;
in float v_mask;

out vec4 out_color;

void main() {
  // (Placeholder code. You will want to replace it.)
  out_color = texture(u_texture_1, v_uv);
  out_color.a = 1;
  out_color = v_mask * out_color;
}
