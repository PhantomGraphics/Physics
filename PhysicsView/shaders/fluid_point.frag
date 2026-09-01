#version 450
layout(location = 0) in float densityValue;
layout(location = 1) flat in float useDensity;
layout(location = 0) out vec4 outColor;

vec3 densityColorMap(float t) {
    const vec3 colors[5] = vec3[5](
        vec3(0.10, 0.20, 0.90), vec3(0.00, 0.85, 1.00),
        vec3(0.10, 0.85, 0.25), vec3(1.00, 0.90, 0.05),
        vec3(0.90, 0.10, 0.05));
    float x = clamp(t, 0.0, 1.0) * 4.0;
    int i = min(int(x), 3);
    return mix(colors[i], colors[i + 1], fract(x));
}

void main() {
    vec3 color = useDensity > 0.5 ? densityColorMap(densityValue)
                                  : vec3(0.2, 0.6, 1.0);
    outColor = vec4(color, 1.0);
}
