#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  mat3 tbh_matrix = mat3(v_tangent, cross(vec3(v_normal), vec3(v_tangent)), v_normal);

  float du = (h(vec2(v_uv.x + 1. / u_texture_2_size.x, v_uv.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dv = (h(vec2(v_uv.x, v_uv.y + 1. / u_texture_2_size.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  vec3 norm = vec3(-1.0 * du, -1.0 * dv, 1.0);

  vec4 new_norm = vec4(tbh_matrix * norm, 0.0);

  float ka = 1.;
  float kd = .5;
  float ks = 1.;
  vec3 Ia = vec3(.2, .2, .2);
  float p = 4.;

  vec3 ambientL = Ia * ka;

  float radius2 = length(v_position.xyz - u_light_pos) * length(v_position.xyz - u_light_pos);
  vec3 diffuseL = kd * (u_light_intensity / radius2) * max(0., dot(normalize(new_norm.xyz), normalize(u_light_pos - v_position.xyz)));
  
  vec3 h = normalize(normalize(u_cam_pos.xyz) + normalize(u_light_pos - v_position.xyz));  
  vec3 specularL = ks * (u_light_intensity / radius2) * pow(max(0., dot(new_norm.xyz, h)), p);

  out_color.rgb = u_color.xyz * (ambientL + diffuseL + specularL);
  out_color.a = 1;
}

