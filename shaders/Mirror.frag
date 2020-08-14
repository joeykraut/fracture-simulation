#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in float v_mask;

out vec4 out_color;

void main() {
  out_color = texture(u_texture_cubemap, vec3(v_normal) - (u_cam_pos - vec3(v_position)));
  out_color.a = 1;
  out_color = v_mask * out_color;
}
