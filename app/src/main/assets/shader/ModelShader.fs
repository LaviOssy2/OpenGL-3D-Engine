#version 300 es
precision mediump float;

in vec2 TexCoords;
out vec4 FragColor;

uniform sampler2D texture_diffuse1;
uniform vec4 color;
uniform bool useTexture;

void main() {
    if (useTexture)
        FragColor = texture(texture_diffuse1, TexCoords) * color;
    else
        FragColor = color;
}
