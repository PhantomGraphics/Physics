#version 450
#extension GL_GOOGLE_include_directive : require
#include "flame_common.glsl"
layout(location=0) in vec4 inPosSize;
layout(location=0) out vec4 outSphere;

// Dual-paraboloid projection covers the whole point-light sphere in two maps.
// The fragment shader ray-tests the actual opaque sphere, including seam rays.
void main() {
    vec3 centre=inPosSize.xyz-ubo.flameLightPosition.xyz;
    float radius=0.5*inPosSize.w;
    float distance=length(centre);
    float z=centre.z*ubo.flameShadow.z;
    outSphere=vec4(centre,radius);
    if (distance<=radius) {
        gl_Position=vec4(0,0,0,1);
        gl_PointSize=2.0*ubo.flameShadow.y;
    } else if (z+radius<0.0) {
        gl_Position=vec4(2,2,2,1); gl_PointSize=1;
    } else {
        vec2 projected=centre.xy/max(distance+z,1e-6);
        // Conservative coverage; exact ray-sphere cutout is in the fragment pass.
        gl_Position=vec4(clamp(projected,vec2(-1),vec2(1)),0,1);
        gl_PointSize=clamp(2.0*radius*ubo.flameShadow.y/max(distance+z-radius,1e-6),1.0,1024.0);
    }
}
