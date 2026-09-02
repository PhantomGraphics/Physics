# Phantom Physics

C++17 による物理シミュレーションモジュール。SPH ベースの流体、剛体、軟体（クロス／ゼリー／ロープ）、
および三者結合（Rigid ↔ Fluid ↔ SoftBody）のソルバー群と、スタンドアロンの Vulkan ビューアを提供する。

> このリポジトリは [`Phantom`](https://github.com/PhantomGraphics/Phantom)
> スーパープロジェクトのサブモジュール（`Physics/`）です。単体でも設定できますが、
> ビルドには同じ親ディレクトリ配下の `CGLib/`（基盤ライブラリ）と `cmake/`（共通ビルドスクリプト）が必要です。

## 構成

| プロジェクト | 種別 | 概要 |
|---|---|---|
| `Physics/` (`PhysicsCore`) | Static library | コアライブラリ。`Phantom::Physics` 名前空間 |
| `PhysicsTest/` | GoogleTest | コアライブラリの単体テスト |
| `PhysicsView/` | Vulkan + ImGui アプリ | 流体・剛体・軟体・結合の統合ビューア。JSON シナリオテスト対応 |
| `Fluid_GPU_Vk/` (`FluidGPUVkCore`) | Static library | Vulkan Compute による GPU 版 CSPH ソルバー |
| `FluidRenderer/` (`FluidRendererCore`) | Static library | Screen Space Fluid Rendering (SSFR) パイプライン |
| `FlameView/` | Vulkan + ImGui アプリ | 燃焼ガス SPH の実験的スタンドアロンビューア（Rigid/Soft 結合なし） |

### コアライブラリの主な内容

- **流体 (SPH):** `ISPHSolver` 共通インターフェースと `DFSPHSolver` / `PBSPHSolver` / `WCSPHSolver` /
  `FlameSolver` 実装。One-Way（SDF ペナルティ）／Two-Way（Akinci 境界粒子）結合、エミッター、流出領域、
  解析境界（Plane / Sphere / Plate）。
- **剛体:** `RigidBody` と `BroadPhase`（BVH）→ `NarrowPhase` → `ContactManifold` の衝突パイプライン、
  `RigidBodySolver`。
- **軟体 (XPBD):** `ClothBody` / `JellyBody` / `RopeBody`、各種 `IConstraint`、`XPBDSolver` / `SoftBodySolver`。
- **三者結合:** `RigidFluidSolver` / `SoftFluidSolver` / `RigidSoftSolver` を束ねる
  トップレベルオーケストレータ `PhysicsSolver`。

より詳しい設計は [`CLAUDE.md`](CLAUDE.md) を参照。

## ビルド

CMake が唯一のビルド手段（C++17 / Vulkan SDK 1.4.341.1 以降 / Ninja）。

```powershell
# 親リポジトリ (Phantom) のルートから、Physics 単体を設定・ビルド
cmake -S Physics -B Physics/build_windows -DCMAKE_BUILD_TYPE=Debug
cmake --build Physics/build_windows

# または Phantom ルートのプリセットでリポジトリ全体を一括ビルド
cmake --preset windows-debug
cmake --build --preset windows-debug
```

ターゲット: `PhysicsCore`, `PhysicsTest`, `PhysicsView`, `FlameView`,
`Fluid_GPU_Vk` (`FluidGPUVkCore`), `FluidRenderer` (`FluidRendererCore`)。

Vulkan ヘッダ／ローダ／GLFW ローダが見つからない場合、`PhysicsView` / `FlameView` は
警告付きでスキップされ、`PhysicsCore` / `PhysicsTest` のみビルドされる（Linux では
`-DVULKAN_INCLUDE_DIR=` / `-DVULKAN_LIBRARY=` / `-DGLFW_LIBRARY=` を渡して有効化）。

SPIR-V シェーダー（`.spv`）は同梱済みのため通常は再コンパイル不要。GLSL を変更した場合のみ
各プロジェクトの `shaders/compile_shaders.bat`（Vulkan SDK の `glslc` を使用）を実行する。

## テスト

すべて GoogleTest（`.exe` として出力）。

```powershell
# 一括ビルド後（build\windows-debug\Physics\ 配下）
.\build\windows-debug\Physics\PhysicsTest.exe

# フィルター例
.\build\windows-debug\Physics\PhysicsTest.exe --gtest_filter=DFSPHSolverTest.*

# ctest 経由（Phantom ルートから）
ctest --preset windows-debug -R PhysicsTest
```

### シナリオテスト（PhysicsView）

`PhysicsView` は JSON シナリオファイルと PowerShell ランナーによる自動テストに対応
（現在 47 本、`NN_<domain>_<name>.json` 命名で `NN` が実行順を兼ねる）。

```powershell
.\Physics\PhysicsView\run_physics_scenarios.ps1 -Configuration Debug

# 一覧表示 / 絞り込み
.\Physics\PhysicsView\run_physics_scenarios.ps1 -List
.\Physics\PhysicsView\run_physics_scenarios.ps1 -Filter '1*_fluid_*'

# 単一シナリオを直接実行
.\build\windows-debug\Physics\PhysicsView.exe --run-scenario Physics\PhysicsView\scenarios\10_fluid_dfsph_pool_settle.json
```

## 技術スタック

C++17 / Vulkan + VulkanMemoryAllocator (VMA) + GLFW / Dear ImGui / GLM 0.9.9.8 + Eigen 3.4.0 /
OpenMP / GoogleTest。

## ライセンス

[MIT License](LICENSE) — Copyright (c) 2026 PhantomGraphics

同梱・依存するサードパーティライブラリ（Dear ImGui, GLM, VulkanMemoryAllocator, Eigen, GoogleTest 等）は
それぞれのライセンスに従います。
