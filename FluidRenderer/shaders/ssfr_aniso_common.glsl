// Shared by ssfr_aniso.vert and ssfr_{depth,thickness}_aniso.frag: ellipsoid splats for the
// Yu & Turk (2013) anisotropic kernel (docs/todo/PLAN_ssfr_anisotropic_kernel.md).
//
// Per particle: inCenter = (smoothed centre, world radius), inAxis0..2 = the
// columns of T = R diag(sigma) (dimensionless, Phantom::Physics::
// toEllipsoidAxes()). The splat is the ellipsoid centre + T_v * unit sphere,
// T_v = mat3(modelView) * T * radius * radiusScale, ray cast per fragment.

layout(set = 0, binding = 0) uniform AnisoUBO {
    mat4 proj;
    mat4 invProj;
    mat4 modelView;
    // x = radius scale, y = viewport width, z = viewport height,
    // w = max point size (device limit)
    vec4 params;
    // x = thickness scale (thickness pass only)
    vec4 params2;
};
