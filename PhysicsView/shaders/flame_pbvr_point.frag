#version 450
#extension GL_GOOGLE_include_directive : require
#include "flame_common.glsl"

layout(location = 0) in vec3 inColor;
layout(location = 1) in vec3 inWorldPosition;
layout(set=0,binding=1) uniform sampler2D shadowFront;
layout(set=0,binding=2) uniform sampler2D shadowBack;
layout(location = 0) out vec4 outColor;

// Hard circular cutout, fully opaque, depth-written: the nearest sub-particle
// wins with no sort (the PBVR premise). Alpha = 1 marks absorbing coverage in
// the ensemble target; the ensemble average of that alpha is 1 - transmittance.
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    if (dot(d, d) > 0.25) {
        discard;
    }
    vec3 color=inColor;
    if (ubo.flameLightPosition.w>0.0) {
        vec3 delta=inWorldPosition-ubo.flameLightPosition.xyz;
        float distance=length(delta);
        vec3 direction=distance>1e-6?delta/distance:vec3(0,0,1);
        vec2 uv=0.5*(direction.xy/(1.0+abs(direction.z))+1.0);
        float nearest=direction.z>=0.0?texture(shadowFront,uv).r:texture(shadowBack,uv).r;
        // Independent shadow particles: no self-shadow bias against the receiver.
        if (distance<=nearest) {
            float radius=ubo.flameLightFlux.w;
            vec3 incident=ubo.flameLightFlux.rgb/(12.5663706*max(distance*distance+radius*radius,1e-8));
            color+=ubo.smokeAlbedo.rgb*incident*ubo.flameLightPosition.w;
        }
    }
    outColor = vec4(color, 1.0);
}
