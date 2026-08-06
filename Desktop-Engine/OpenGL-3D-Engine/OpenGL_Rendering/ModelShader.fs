#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture_diffuse1;
uniform vec4 color;  // Mesh color
uniform bool useTexture; // Control texture usage

void main()
{
    if (useTexture)
        FragColor = texture(texture_diffuse1, TexCoords) * color;
    else
        FragColor = color; // Fallback to uniform color
}