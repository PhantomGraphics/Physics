#version 450

layout(location = 0) in  vec4  vViewPos;
layout(location = 1) in  float vParticleRadius;
layout(location = 0) out float outDepth;

layout(set = 0, binding = 0) uniform DepthUBO {
    mat4  proj;
    mat4  modelView;
    float particleRadius;
    float viewportHeight;
};

void main() {
    // ポイントスプライト中心からの正規化座標 [-1,1]
    vec2 coord = gl_PointCoord * 2.0 - 1.0;
    float r = dot(coord, coord);

    // 円の外側を破棄（球形クリッピング）
    if (r > 1.0) discard;

    // 球面 z 成分（単位球の表面）
    float zSphere = sqrt(1.0 - r);

    // ビュー空間での球面点の近似
    // vPointSize はスクリーンピクセル単位だが、OpenGL版と同一の近似を踏襲
    vec4 surfPos = vViewPos + vParticleRadius * vec4(coord, zSphere, 0.0);

    // クリップ空間へ投影し、Vulkan NDC 深度 [0,1] を計算
    // GLM_FORCE_DEPTH_ZERO_TO_ONE が有効なので proj は既に [0,1] 範囲を出力する
    vec4 clip = proj * surfPos;
    outDepth = clip.z / clip.w;
    // The hardware depth attachment must use the reconstructed sphere surface
    // too; otherwise overlapping splats are selected by their centre depth.
    gl_FragDepth = outDepth;
}
