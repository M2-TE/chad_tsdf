#version 460

layout(location = 0) in vec3 in_normal;
layout(location = 1) in vec3 in_color;
layout(location = 0) out vec4 out_color;

void main() {
    vec3 light_dir = vec3(0.0, 0.0, -1.0);
    float intensity = max(dot(in_normal, light_dir), 0.0);
    out_color = vec4(in_color * intensity, 1.0);
}