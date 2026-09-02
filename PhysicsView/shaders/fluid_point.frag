#version 450
layout(location = 0) in float densityValue;
layout(location = 1) flat in float useDensity;
layout(location = 0) out vec4 outColor;

vec3 densityColorMap(float t) {
    // Diverging map: below rest density is blue, rest density is neutral,
    // and above rest density is red.
    const vec3 low = vec3(0.10, 0.35, 0.95);
    const vec3 neutral = vec3(0.95, 0.95, 0.95);
    const vec3 high = vec3(0.90, 0.10, 0.05);
    float x = clamp(t, 0.0, 1.0);
    return x < 0.5 ? mix(low, neutral, x * 2.0)
                   : mix(neutral, high, (x - 0.5) * 2.0);
}

void main() {
    vec3 color = useDensity > 0.5 ? densityColorMap(densityValue)
                                  : vec3(0.2, 0.6, 1.0);
    outColor = vec4(color, 1.0);
}
