// Shared by every Flame page shader (#include "flame_common.glsl").
// Mirrors FlamePointUBO in FlamePointPipeline.h -- keep the two in sync.
//
// Emission / absorption model (docs/todo/PLAN_flame_sph_pbvr_improvement.md
// Phase 3): hot gas *emits* blackbody light (additive, never occludes), soot
// smoke *absorbs* (optical depth sigma * density), and hot soot also glows in
// proportion to what it absorbs (Kirchhoff). Everything is linear HDR; the
// PhysicsView SSFR composite applies ACES + exposure once at the end.

layout(set = 0, binding = 0) uniform FlameUBO {
    mat4 mvp;
    vec4 view;        // x = |proj[1][1]|, y = viewport height (px), z = min PBVR sub-particle size (px), w = PBVR density scale
    vec4 thermal;     // x = T_ambient, y = T_ref (radiance 1), z = flame exposure, w = unused
    vec4 smoke;       // x = extinction sigma (per unit soot density), y = smoke glow scale, z = PBVR subdivision, w = unused
    vec4 smokeAlbedo; // rgb = soot albedo * ambient light
    vec4 lutRange;    // x = LUT min T, y = LUT max T
    vec4 lut[64];     // blackbody chromaticity (unit luminance, linear sRGB), see FlameBlackbody.h
} ubo;

vec3 flameBlackbody(float T) {
    float u = clamp((T - ubo.lutRange.x) / max(ubo.lutRange.y - ubo.lutRange.x, 1.0), 0.0, 1.0) * 63.0;
    int i = int(floor(u));
    int j = min(i + 1, 63);
    return mix(ubo.lut[i].rgb, ubo.lut[j].rgb, u - float(i));
}

// Stefan-Boltzmann relative radiant exitance, 1 at T_ref, > 1 above it (HDR).
float flameRelativeRadiance(float T) {
    float ta2 = ubo.thermal.x * ubo.thermal.x;
    float tr2 = ubo.thermal.y * ubo.thermal.y;
    float t2 = T * T;
    return max(0.0, (t2 * t2 - ta2 * ta2) / max(tr2 * tr2 - ta2 * ta2, 1.0));
}

vec3 flameEmission(float T) {
    return flameBlackbody(T) * flameRelativeRadiance(T) * ubo.thermal.z;
}

// Colour an absorbing (smoke) sample re-emits per unit of its own opacity:
// ambient-lit soot albedo plus its thermal glow.
vec3 smokeColor(float T) {
    return ubo.smokeAlbedo.rgb + flameEmission(T) * ubo.smoke.y;
}

// World-space diameter -> gl_PointSize with perspective (plan A7).
float flameProjectedSize(float worldDiameter, vec4 clipPos) {
    return worldDiameter * ubo.view.x * ubo.view.y / (2.0 * max(clipPos.w, 1.0e-4));
}
float flamePointSize(float worldDiameter, vec4 clipPos) {
    return clamp(flameProjectedSize(worldDiameter, clipPos), 1.0, 1024.0);
}
