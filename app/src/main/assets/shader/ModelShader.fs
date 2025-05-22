#version 300 es
precision mediump float;

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 lightColor;

uniform vec3 lightDir; // This should be normalized
uniform vec3 viewPos;
uniform vec4 color;
uniform bool useTexture;
uniform sampler2D texture_diffuse1;

void main() {
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, -lightDir), 0.0); // NEGATE for light *coming from* dir
    vec3 diffuse = diff * lightColor;

    vec3 ambient = 0.2 * lightColor;

    vec3 result = (ambient + diffuse) * color.rgb;
    FragColor = vec4(result, color.a);
}
