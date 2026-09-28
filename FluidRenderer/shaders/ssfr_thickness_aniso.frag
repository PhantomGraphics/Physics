#version 450
#extension GL_GOOGLE_include_directive : require

#include "ssfr_aniso_common.glsl"
#include "ssfr_aniso_raycast.glsl"

layout(location = 0) flat in vec3 vCenter;
layout(location = 1) flat in mat3 vInvAxes;
layout(location = 0) out float outThickness;

void main() {
    vec3 origin, dir;
    float t0, t1;
    if (!raycastEllipsoid(vCenter, vInvAxes, origin, dir, t0, t1)) discard;

    // Chord length through the ellipsoid (ssfr_thickness.frag's 2 r z for a sphere).
    outThickness = (t1 - t0) * length(dir) * params2.x;
}
