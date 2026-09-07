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
    int passAxis;
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

    // UI sigma values historically came from the old two-pixel kernel.  Treat
    // them as half-resolution units so the footprint also covers a projected
    // particle at close camera distances.
    float filterSigma = max(sigmaS * 2.0, 0.001);
    // Raw particle thickness has steep, periodic gradients.  The previous
    // unconstrained response reached 7x in the high preset, narrowed the
    // cross-gradient kernel below one pixel, and preserved every particle
    // column.  A modest cap keeps coherent silhouettes directional without
    // treating the splat lattice as geometry.
    float anisoGain = min(1.0 + anisotropy * clamp(gradMag * gradientScale, 0.0, 4.0),
                          1.25);
    float sigmaT = filterSigma * anisoGain;
    float sigmaN = filterSigma / anisoGain;
    sigmaN = max(sigmaN, 0.35);

    float sum = 0.0;
    float wsum = 0.0;

    if (passAxis != 0) {
        // Truncate at 3 sigma (capped for predictable real-time cost).  The old
        // fixed radius of two pixels was narrower than sigmaS itself in the
        // high preset and preserved the projected particle lattice.
        int radius = clamp(int(ceil(3.0 * filterSigma)), 1, 16);
        for (int i = -16; i <= 16; ++i) {
            if (abs(i) > radius) continue;
            vec2 pixelOffset = passAxis == 1 ? vec2(float(i), 0.0)
                                             : vec2(0.0, float(i));
            vec2 offset = pixelOffset * texelSize;
            float sampleV = texture(uThickness, vUV + offset).r;
            float ws;
            if (useAnisotropic != 0) {
                float dt = dot(pixelOffset, t);
                float dn = dot(pixelOffset, n);
                float et = (dt * dt) / (2.0 * sigmaT * sigmaT);
                float en = (dn * dn) / (2.0 * sigmaN * sigmaN);
                ws = exp(-(et + en));
            } else {
                ws = gaussian(abs(float(i)), filterSigma);
            }
            float wr = gaussian(sampleV - center, sigmaR);
            float w = ws * wr;
            sum += sampleV * w;
            wsum += w;
        }
    } else {
        // Compatibility fallback for callers that have not selected an axis.
        for (int y = -2; y <= 2; ++y) {
            for (int x = -2; x <= 2; ++x) {
                vec2 offset = vec2(float(x), float(y)) * texelSize;
                float sampleV = texture(uThickness, vUV + offset).r;
                float ws = gaussian(length(vec2(x, y)), sigmaS);
                float wr = gaussian(sampleV - center, sigmaR);
                float w = ws * wr;
                sum += sampleV * w;
                wsum += w;
            }
        }
    }

    outThickness = (wsum > 0.0) ? (sum / wsum) : center;
}
