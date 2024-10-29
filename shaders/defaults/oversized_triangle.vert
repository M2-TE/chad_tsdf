#version 460

// draw oversized triangle to achieve fullscreen fragment shader
void main() {
    int id = gl_VertexIndex;
    gl_Position.x = id == 1 ? 3.0 : -1.0;
    gl_Position.y = id == 2 ? 3.0 : -1.0;
    gl_Position.zw = vec2(0.0, 1.0);
    // out_texcoord.x = id == 1 ? 2.0 : 0.0;
    // out_texcoord.y = id == 2 ? 2.0 : 0.0;
}