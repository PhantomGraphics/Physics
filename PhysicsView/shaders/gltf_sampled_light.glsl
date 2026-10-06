layout(set=2,binding=0) uniform sampler2D particleShadowFront;
layout(set=2,binding=1) uniform sampler2D particleShadowBack;
layout(push_constant) uniform SampledLight {
    layout(offset=64) vec4 positionGain;
    vec4 fluxRadius;
} sampledLight;

vec3 sampledDiffuseLight(vec3 position, vec3 normal, vec3 view, vec3 albedo, float metallic) {
    if (sampledLight.positionGain.w<=0.0) return vec3(0);
    vec3 fromLight=position-sampledLight.positionGain.xyz;
    float distance=length(fromLight);
    vec3 direction=distance>1e-6?fromLight/distance:vec3(0,0,1);
    float cosine=max(dot(normal,-direction),0.0);
    if (cosine<=0.0) return vec3(0);
    vec2 uv=0.5*(direction.xy/(1.0+abs(direction.z))+1.0);
    float nearest=direction.z>=0.0?texture(particleShadowFront,uv).r:texture(particleShadowBack,uv).r;
    if (distance>nearest) return vec3(0);
    float radius=sampledLight.fluxRadius.w;
    vec3 incident=sampledLight.fluxRadius.rgb*sampledLight.positionGain.w/
        (12.5663706*max(distance*distance+radius*radius,1e-8));
    // Diffuse only. Retain material Fresnel/metallic energy partition; normal
    // maps and base-color textures are supplied by the regular glTF shader.
    vec3 F0=mix(vec3(0.04),albedo,metallic);
    vec3 halfVector=view-direction;
    float halfLength=length(halfVector);
    float viewHalf=halfLength>1e-6?max(dot(view,halfVector/halfLength),0.0):0.0;
    vec3 fresnel=F0+(1.0-F0)*pow(1.0-viewHalf,5.0);
    return (1.0-fresnel)*(1.0-metallic)*albedo*incident*(cosine/3.14159265);
}
