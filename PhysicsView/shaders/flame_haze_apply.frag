#version 450

// Heat haze: re-draws the background scene into the HDR scene target with a
// heat-gradient and evolving turbulence displacement of the *colour* (depth is copied
// unchanged, see the end of main()). Everything is
// fetched by texel (nearest sampler); colour and the haze field are filtered by
// hand so the source pixel can move by sub-pixel amounts.
layout(set = 0, binding = 0) uniform sampler2D bgColor;
layout(set = 0, binding = 1) uniform sampler2D bgDepth;
layout(set = 0, binding = 2) uniform sampler2D hazeField;

layout(push_constant) uniform Push {
    vec4 a; // x = strength (px), y = time (s), z = noise frequency (cells per screen height), w = rise speed (screen heights / s)
} pc;

layout(location = 0) out vec4 outColor;

float hash(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

float valueNoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
               mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}

vec2 noise2(vec2 p) {
    return vec2(valueNoise(p), valueNoise(p + vec2(31.7, 17.3))) - 0.5;
}

vec4 fetchBilinear(sampler2D s, vec2 pixel) {
    ivec2 size = textureSize(s, 0);
    vec2 q = pixel - 0.5;
    ivec2 i0 = ivec2(floor(q));
    vec2 f = q - vec2(i0);
    ivec2 lo = clamp(i0, ivec2(0), size - 1);
    ivec2 hi = clamp(i0 + 1, ivec2(0), size - 1);
    vec4 a = mix(texelFetch(s, ivec2(lo.x, lo.y), 0), texelFetch(s, ivec2(hi.x, lo.y), 0), f.x);
    vec4 b = mix(texelFetch(s, ivec2(lo.x, hi.y), 0), texelFetch(s, ivec2(hi.x, hi.y), 0), f.x);
    return mix(a, b, f.y);
}

float heatAt(vec2 pixel) {
    float h = max(fetchBilinear(hazeField, pixel).r, 0.0);
    return h / (1.0 + h);
}

void main() {
    ivec2 size = textureSize(bgColor, 0);
    ivec2 own = ivec2(gl_FragCoord.xy);
    float d0 = texelFetch(bgDepth, own, 0).r;

    // Heat at this pixel (field is low-res: map by normalised position).
    vec2 uv = gl_FragCoord.xy / vec2(size);
    vec2 fieldPixel = uv * vec2(textureSize(hazeField, 0));
    vec4 field = fetchBilinear(hazeField, fieldPixel);
    float heat = field.r / (1.0 + field.r);              // soft saturation
    float flameDepth = field.g / max(field.r, 1.0e-5);
    // Geometry in front of the hot gas is not seen through it.
    float behind = smoothstep(flameDepth, flameDepth + 0.0005, d0);

    vec2 offset = vec2(0.0);
    if (heat > 1.0e-4 && behind > 0.0) {
        float aspect = float(size.x) / float(size.y);
        vec2 p = vec2(uv.x * aspect, uv.y) * pc.a.z;
        p.y += pc.a.y * pc.a.w * pc.a.z;                 // pattern rises with time
        // Domain warping lets eddies deform as they rise instead of sliding a
        // fixed screen-space pattern. Use the simulation clock: pause freezes it.
        float phase = pc.a.y * pc.a.w;
        vec2 warp = noise2(p * 0.47 + vec2(phase * 0.31, -phase * 0.19));
        vec2 n = noise2(p + warp * 1.7) +
                 0.5 * noise2(p * 2.1 + vec2(-phase * 0.73, phase * 0.41) + 7.7);
        // Refraction responds to the transverse temperature gradient, with
        // smaller turbulent eddies inside the hot plume. Saturated heat keeps
        // overlapping SPH splats from producing unbounded displacements.
        vec2 gradient = vec2(
            heatAt(fieldPixel + vec2(1, 0)) - heatAt(fieldPixel - vec2(1, 0)),
            heatAt(fieldPixel + vec2(0, 1)) - heatAt(fieldPixel - vec2(0, 1))) * 0.5;
        vec2 distortion = gradient * 3.0 + n * vec2(1.6, 0.9) * heat;
        offset = distortion / max(1.0, length(distortion)) * pc.a.x * behind;
    }

    vec2 srcPixel = clamp(gl_FragCoord.xy + offset, vec2(0.5), vec2(size) - 0.5);
    float d1 = texelFetch(bgDepth, ivec2(srcPixel), 0).r;
    // Reject foreground silhouettes, not every negative depth slope. The old
    // d1 < d0 test suppressed shimmer across ordinary tilted background faces.
    if (field.r > 1.0e-5 && d1 <= flameDepth + 0.0005) {
        srcPixel = gl_FragCoord.xy;
    }
    outColor = fetchBilinear(bgColor, srcPixel);
    // Depth stays undisplaced: the flame (and SSFR) depth-test against the true
    // geometry; only what is *seen* through the hot gas is refracted.
    gl_FragDepth = d0;
}
