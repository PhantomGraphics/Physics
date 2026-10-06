#version 450
#extension GL_GOOGLE_include_directive : require
#include "flame_common.glsl"
layout(location=0) in vec4 inSphere;
layout(location=0) out vec4 outDepth;
void main() {
    vec2 q=2.0*gl_FragCoord.xy/ubo.flameShadow.y-1.0;
    float q2=dot(q,q);
    if (q2>1.0) discard;
    vec3 direction=vec3(2.0*q,ubo.flameShadow.z*(1.0-q2))/(1.0+q2);
    vec3 centre=inSphere.xyz;
    float radius=inSphere.w;
    float along=dot(centre,direction);
    float discriminant=radius*radius-dot(centre,centre)+along*along;
    if (discriminant<0.0) discard;
    float halfChord=sqrt(discriminant);
    if (along+halfChord<0.0) discard;
    float nearest=max(0.0,along-halfChord);
    gl_FragDepth=nearest/ubo.flameShadow.x;
    outDepth=vec4(nearest,0,0,1);
}
