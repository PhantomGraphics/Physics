// Ray / ellipsoid intersection for the anisotropic splats. Returns false when
// the fragment's view ray misses; t0/t1 are the entry/exit parameters along
// `dir` from `origin` (both view space).
//
// `origin` is just some point on the pixel's view ray (NDC z = 0), not the
// eye: with a [-1,1] depth-range projection NDC z = 0 lies mid-frustum, so t
// may legitimately be negative. Only the ordering matters -- `dir` always
// points away from the eye (NDC z grows with distance in both conventions).
bool raycastEllipsoid(vec3 center, mat3 invAxes, out vec3 origin, out vec3 dir,
                      out float t0, out float t1)
{
    vec2 ndc = vec2(gl_FragCoord.x / params.y, gl_FragCoord.y / params.z) * 2.0 - 1.0;
    vec4 pNear = invProj * vec4(ndc, 0.0, 1.0);
    vec4 pMid  = invProj * vec4(ndc, 0.5, 1.0);
    origin = pNear.xyz / pNear.w;
    dir    = pMid.xyz / pMid.w - origin; // valid for perspective and orthographic

    vec3 o = invAxes * (origin - center);
    vec3 d = invAxes * dir;
    float a = dot(d, d);
    float b = dot(o, d);
    float c = dot(o, o) - 1.0;
    float disc = b * b - a * c;
    if (disc < 0.0 || a <= 0.0) return false;
    float s = sqrt(disc);
    t0 = (-b - s) / a;
    t1 = (-b + s) / a;
    return true;
}
