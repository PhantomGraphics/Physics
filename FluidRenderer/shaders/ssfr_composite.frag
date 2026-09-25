#version 450
layout(location=0) in vec2 vUV;
layout(location=0) out vec4 outColor;
layout(set=0,binding=0) uniform sampler2D uDepth;
layout(set=0,binding=1) uniform sampler2D uThicknessRaw;
layout(set=0,binding=2) uniform sampler2D uThicknessSmooth;
layout(set=0,binding=3) uniform sampler2D uReflection;
layout(set=0,binding=4) uniform sampler2D uRefraction;
layout(set=0,binding=5) uniform sampler2D uSpray;
layout(set=0,binding=6) uniform sampler2D uFoam;
layout(set=0,binding=7) uniform Composite {
    int mode; float foamOpacity; float sprayOpacity; int showSpray;
    int showFoam; int hasScene; float exposure; int transparent;
    vec4 absorptionColor;
    float absorptionDistance; float thicknessScale; float ior; float roughness;
};
layout(set=0,binding=8) uniform sampler2D uSceneColor;
layout(set=0,binding=9) uniform sampler2D uSceneDepth;

vec3 aces(vec3 x) {
    if ((transparent & 2) != 0) return x;
    const float a=2.51,b=0.03,c=2.43,d=0.59,e=0.14;
    return clamp((x*(a*x+b))/(x*(c*x+d)+e),0.0,1.0);
}
void main() {
    // transparent != 0: emit an RGBA matte -- alpha follows coverage (opaque
    // scene geometry via the HDR target's alpha, fluid via the depth/thickness
    // test) so the sequence composites over a plate. Otherwise alpha is 1.
    vec4 sceneTex = hasScene != 0 ? texture(uSceneColor,vUV) : vec4(0.0);
    vec3 scene = sceneTex.rgb;
    // Universe scene color already includes exposure. Apply it only once.
    if ((transparent & 2) != 0) scene /= max(exposure, 0.001);
    float sceneA = sceneTex.a;
    float bgA = (transparent & 1) != 0 ? sceneA : 1.0;
    float sceneDepth=hasScene!=0?texture(uSceneDepth,vUV).r:1.0;
    gl_FragDepth = sceneDepth;
    if (mode == -1) { outColor=vec4(aces(scene*exposure),bgA); return; }
    float depth=texture(uDepth,vUV).r;
    float raw=texture(uThicknessRaw,vUV).r;
    float thick=texture(uThicknessSmooth,vUV).r;
    bool visible=depth>0.0 && (hasScene==0 || depth<sceneDepth-0.00005);
    if (visible && thick > 0.0001) gl_FragDepth = depth;
    float coverA = (transparent & 1) != 0 ? (visible ? 1.0 : sceneA) : 1.0;
    if (mode==0 || mode==6) { vec3 c=visible?vec3(depth):scene; outColor=vec4(aces(c*exposure),coverA); return; }
    if (mode==1) { vec3 c=visible?vec3(raw*0.3):scene; outColor=vec4(aces(c*exposure),coverA); return; }
    if (mode==2) { vec3 c=visible?vec3(thick*0.3):scene; outColor=vec4(aces(c*exposure),coverA); return; }
    if (!visible || thick<=0.0001) { outColor=vec4(aces(scene*exposure),bgA); return; }
    vec4 reflectionSample=texture(uReflection,vUV);
    vec3 refl=reflectionSample.rgb;
    vec3 refr=texture(uRefraction,vUV).rgb;
    if ((transparent & 2) != 0 && hasScene != 0) refr /= max(exposure, 0.001);
    if(mode==3){outColor=vec4(aces(refl*exposure),coverA);return;}
    if(mode==4){outColor=vec4(aces(refr*exposure),coverA);return;}
    float scaledThickness=max(thick*thicknessScale,0.0);
    vec3 transmittance=exp(-max(vec3(0.0),vec3(1.0)-absorptionColor.rgb)*scaledThickness/max(absorptionDistance,0.001));
    // reflectionSample.a carries N.V from the filtered surface normal.  The
    // previous thickness-derived cosine made every deep region a perfect
    // mirror, which is why water rendered as an opaque blue slab.
    float cosTheta=clamp(reflectionSample.a,0.0,1.0);
    float f0=pow((ior-1.0)/(ior+1.0),2.0);
    float grazing=max(1.0-clamp(roughness,0.0,1.0),f0);
    float fresnel=f0+(grazing-f0)*pow(1.0-cosTheta,5.0);
    // A small single-scattering approximation keeps optically deep water from
    // collapsing to black after Beer-Lambert extinction while thin sheets stay
    // dominated by the actually refracted scene.
    vec3 transmitted=refr*transmittance+
                     absorptionColor.rgb*(vec3(1.0)-transmittance)*0.10;
    vec3 color=mix(transmitted,refl,clamp(fresnel,0.0,0.98));
    if(showFoam!=0) color=mix(color,vec3(0.95),clamp(texture(uFoam,vUV).r*foamOpacity,0.0,1.0));
    if(showSpray!=0) color+=vec3(0.95)*clamp(texture(uSpray,vUV).r*sprayOpacity,0.0,1.0);
    outColor=vec4(aces(color*exposure),coverA);
}
