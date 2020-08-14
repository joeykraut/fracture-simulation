#version 330

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in float v_mask;

out vec4 out_color;

void main() {
  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
  out_color = v_mask * out_color;
}
