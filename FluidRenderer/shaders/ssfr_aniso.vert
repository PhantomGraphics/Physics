#version 450
#extension GL_GOOGLE_include_directive : require

#include "ssfr_aniso_common.glsl"

layout(location = 0) in vec4 inCenter;
layout(location = 1) in vec4 inAxis0;
layout(location = 2) in vec4 inAxis1;
layout(location = 3) in vec4 inAxis2;

layout(location = 0) flat out vec3 vCenter;   // view space
layout(location = 1) flat out mat3 vInvAxes;  // view space -> unit sphere

void main() {
    float r = inCenter.w * params.x;
    mat3 axes = mat3(modelView) * mat3(inAxis0.xyz, inAxis1.xyz, inAxis2.xyz) * r;
    vec4 c = modelView * vec4(inCenter.xyz, 1.0);
    vCenter = c.xyz;
    vInvAxes = inverse(axes);
    gl_Position = proj * c;

    // Conservative sprite: the ellipsoid lies inside the sphere of its longest
    // semi-axis. The small margin covers the perspective offset between the
    // projected centre and the centre of the projected ellipse.
    float bound = max(length(axes[0]), max(length(axes[1]), length(axes[2])));
    float viewDistance = abs(proj[3][3]) > 0.5 ? 1.0 : max(-c.z, 1.0e-4);
    float size = 1.15 * bound * abs(proj[1][1]) * params.z / viewDistance + 2.0;
    gl_PointSize = clamp(size, 1.0, params.w);
}
