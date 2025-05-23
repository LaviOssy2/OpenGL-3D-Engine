#version 300 es
precision mediump float;

struct DirectionalLight {
    vec3 direction;
    vec3 color;
};

struct PointLight {
    vec3 position;
    vec3 color;
    float constant;
    float linear;
    float quadratic;
};

struct SpotLight {
    vec3 position;
    vec3 direction;
    float cutOff;
    float outerCutOff;
    vec3 color;
    float constant;
    float linear;
    float quadratic;
};

#define MAX_POINT_LIGHTS 4
#define MAX_SPOT_LIGHTS  4

uniform DirectionalLight dirLight;
uniform PointLight      pointLights[MAX_POINT_LIGHTS];
uniform int             numPointLights;
uniform SpotLight       spotLights[MAX_SPOT_LIGHTS];
uniform int             numSpotLights;

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

out vec4 FragColor;

// forward declarations
vec3 CalcDirLight(DirectionalLight light, vec3 normal);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos);
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos);

void main() {
    vec3 norm   = normalize(Normal);
    vec3 result = vec3(0.0);

    // Directional light
    result += CalcDirLight(dirLight, norm);

    // Point lights
    for (int i = 0; i < numPointLights; ++i)
        result += CalcPointLight(pointLights[i], norm, FragPos);

    // Spot lights
    for (int i = 0; i < numSpotLights; ++i)
        result += CalcSpotLight(spotLights[i], norm, FragPos);

    FragColor = vec4(result, 1.0);
}

vec3 CalcDirLight(DirectionalLight light, vec3 normal) {
    vec3 lightDir = normalize(-light.direction);
    float diff    = max(dot(normal, lightDir), 0.0);
    return light.color * diff;
}

vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos) {
    vec3 lightDir = normalize(light.position - fragPos);
    float diff    = max(dot(normal, lightDir), 0.0);
    float dist    = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * dist + light.quadratic * dist * dist);
    return light.color * diff * attenuation;
}

vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos) {
    vec3 lightDir = normalize(light.position - fragPos);
    float theta   = dot(lightDir, normalize(-light.direction));
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);
    float diff    = max(dot(normal, lightDir), 0.0);
    float dist    = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * dist + light.quadratic * dist * dist);
    return light.color * diff * attenuation * intensity;
}