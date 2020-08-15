#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;
in float v_mask;

out vec4 out_color;

void main() {
  // YOUR CODE HERE

  float ka = 1.;
  float kd = .5;
  float ks = 1.;
  vec3 Ia = vec3(.2, .2, .2);
  float p = 4.;

  vec3 ambientL = Ia * ka;

  float radius2 = length(v_position.xyz - u_light_pos) * length(v_position.xyz - u_light_pos);
  vec3 diffuseL = kd * (u_light_intensity / radius2) * max(0., dot(normalize(v_normal.xyz), normalize(u_light_pos - v_position.xyz)));
  
  vec3 h = normalize(normalize(u_cam_pos.xyz) + normalize(u_light_pos - v_position.xyz));  
  vec3 specularL = ks * (u_light_intensity / radius2) * pow(max(0., dot(v_normal.xyz, h)), p);

  out_color.rgb = u_color.xyz * (ambientL + diffuseL + specularL);
  out_color.a = 1;
  out_color = v_mask * out_color;
}

