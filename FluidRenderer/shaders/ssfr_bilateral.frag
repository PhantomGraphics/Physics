#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out float outThickness;

layout(set = 0, binding = 0) uniform sampler2D uThickness;
layout(set = 0, binding = 1) uniform Params {
    vec2 texelSize;
    float sigmaS;
    float sigmaR;
    int useAnisotropic;
    float anisotropy;
    float gradientScale;
    float _pad;
};

float gaussian(float x, float sigma) {
    return exp(-(x * x) / (2.0 * sigma * sigma));
}

void main() {
    float center = texture(uThickness, vUV).r;
    if (center <= 0.0001) {
        outThickness = center;
        return;
    }

    float rightV = texture(uThickness, vUV + vec2(texelSize.x, 0.0)).r;
    float leftV  = texture(uThickness, vUV - vec2(texelSize.x, 0.0)).r;
    float upV    = texture(uThickness, vUV + vec2(0.0, texelSize.y)).r;
    float downV  = texture(uThickness, vUV - vec2(0.0, texelSize.y)).r;

    vec2 grad = vec2(rightV - leftV, upV - downV);
    float gradMag = length(grad);

    vec2 n = (gradMag > 1e-5) ? (grad / gradMag) : vec2(0.0, 1.0);
    vec2 t = vec2(-n.y, n.x);

    float anisoGain = 1.0 + anisotropy * clamp(gradMag * gradientScale, 0.0, 4.0);
    float sigmaT = sigmaS * anisoGain;
    float sigmaN = sigmaS / anisoGain;
    sigmaN = max(sigmaN, 0.35);

    float sum = 0.0;
    float wsum = 0.0;

    for (int y = -2; y <= 2; ++y) {
        for (int x = -2; x <= 2; ++x) {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            float sampleV = texture(uThickness, vUV + offset).r;
            float ws;
            if (useAnisotropic != 0) {
                vec2 p = vec2(float(x), float(y));
                float dt = dot(p, t);
                float dn = dot(p, n);
                float et = (dt * dt) / (2.0 * sigmaT * sigmaT);
                float en = (dn * dn) / (2.0 * sigmaN * sigmaN);
                ws = exp(-(et + en));
            } else {
                ws = gaussian(length(vec2(x, y)), sigmaS);
            }
            float wr = gaussian(sampleV - center, sigmaR);
            float w = ws * wr;
            sum += sampleV * w;
            wsum += w;
        }
    }

    outThickness = (wsum > 0.0) ? (sum / wsum) : center;
}
