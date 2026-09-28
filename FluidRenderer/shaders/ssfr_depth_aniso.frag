#version 450
#extension GL_GOOGLE_include_directive : require

#include "ssfr_aniso_common.glsl"
#include "ssfr_aniso_raycast.glsl"

layout(location = 0) flat in vec3 vCenter;
layout(location = 1) flat in mat3 vInvAxes;
layout(location = 0) out float outDepth;

void main() {
    vec3 origin, dir;
    float t0, t1;
    if (!raycastEllipsoid(vCenter, vInvAxes, origin, dir, t0, t1)) discard;

    vec4 clip = proj * vec4(origin + t0 * dir, 1.0);
    outDepth = clip.z / clip.w;
    // As in ssfr_depth.frag: the hardware depth must be the surface, not the
    // splat centre, so overlapping ellipsoids resolve per pixel.
    gl_FragDepth = outDepth;
}
