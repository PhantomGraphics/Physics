#pragma once

#include <vector>
#include "IConstraint.h"
#include "CGLib/Math/Matrix3d.h"

namespace Phantom {
namespace Physics {

// 形状一致制約 (Müller et al. 2005 "Meshless Deformations Based on Shape Matching")
// 現在の重心を計算し、SVD（回転抽出）で目標位置を定め、各粒子を引き寄せる。
// 一括制約（全 indices を同時に扱う）のため IConstraint の alphaHat は
// ブレンド係数に変換して使用。
struct ShapeMatchingConstraint : IConstraint {
    std::vector<int>             indices;   // 対象粒子インデックス
    std::vector<Math::Vector3df> restPos;   // 重心相対の初期位置
    std::vector<float>           masses;    // 各粒子の質量（= 1/inverseMass; ピン=0）

    // インデックスと初期粒子配列から制約を構築する
    void build(const std::vector<int>& idx, const SoftParticleSoA& particles);

    void project(SoftParticleSoA& particles, float alphaHat) override;

private:
    // Müller 2016 "A Robust Method to Extract the Rotational Part of Deformations"
    // 反復回転抽出（最大 10 反復）
    static Math::Matrix3df extractRotation(const Math::Matrix3df& Apq, int maxIter = 10);
};

} // namespace Physics
} // namespace Phantom
