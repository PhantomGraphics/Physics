# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Overview

流体・剛体・軟体（クロス／ゼリー／ロープ）シミュレーションと SPH ベースの三者結合（Rigid↔Fluid↔SoftBody）を提供するモジュール群。
`Physics`（コアライブラリ）、`PhysicsTest`（GoogleTest）、`PhysicsView`（スタンドアロン ImGui + Vulkan ビューア。炎 SPH を含む——下記 Flame 節）、`Fluid_GPU_Vk`（GPU Compute CSPH）、`FluidRenderer`（Screen Space Fluid Rendering）の 5 プロジェクトで構成される。単独の `.sln` は持たず、すべて上位の `Phantom2026.sln` でビルドする。旧 `FlameView`（炎 SPH の独立スタンドアロンビューア）は 2026-09-08 に PhysicsView へ統合済み。

親リポジトリの AGENTS.md（`../AGENTS.md`）にビルド方法・全体アーキテクチャ・命名規則が記載されているのであわせて参照すること。

## Build

CMake が唯一のビルド手段（2026-08-19、`.vcxproj` は全削除済み。詳細は
内部設計メモ Phase 5、親リポジトリ `../AGENTS.md` の Build 節を参照）。

```powershell
# リポジトリルートから、Physics単体を設定・ビルド
cmake -S Physics -B Physics/build_windows -DCMAKE_BUILD_TYPE=Debug
cmake --build Physics/build_windows

# または、ルートの CMakePresets.json 経由でリポジトリ全体を一括ビルド
cmake --preset windows-debug
cmake --build --preset windows-debug
```

ターゲット: `PhysicsCore`, `PhysicsTest`, `PhysicsView`, `Fluid_GPU_Vk`(`FluidGPUVkCore`),
`FluidRenderer`(`FluidRendererCore`)。実行ファイル名は CMake ターゲット名基準で `PhysicsView.exe`
（`RootNamespace` が旧名 `VkFluidView` だった vcxproj 時代の名残はもう関係ない）。

## Tests

```powershell
# ルートから一括ビルドした場合（build\windows-debug\Physics\ 配下）
.\build\windows-debug\Physics\PhysicsTest.exe

# フィルター例
.\build\windows-debug\Physics\PhysicsTest.exe --gtest_filter=PhysicsSolverTest.*
.\build\windows-debug\Physics\PhysicsTest.exe --gtest_filter=DFSPHSolverTest.*

# ctest 経由（リポジトリルートから）
ctest --preset windows-debug -R PhysicsTest
```

`PhysicsTest/PhysicsFluidFactory.h/.cpp` はテスト専用ヘルパー（`ISPHSolver` 実装をシナリオ別に構築する）。`Physics` 本体には存在しないので、プロダクションコードから参照しない。

### シナリオテスト（PhysicsView）

内部設計メモ に基づき全面再構築済み（2026-08）。ランナーは
`run_fluid_scenarios.ps1` から `run_physics_scenarios.ps1` へリネーム（対象が流体単体ではなく
PhysicsView 全体——fluid + rigid + soft-body + coupling——であるため）。

```powershell
.\Physics\PhysicsView\run_physics_scenarios.ps1 -Configuration Debug

# フィルター・タグ絞り込み・一覧表示
.\Physics\PhysicsView\run_physics_scenarios.ps1 -Filter '1*_fluid_*'   # 番号帯プレフィックスで絞り込み
.\Physics\PhysicsView\run_physics_scenarios.ps1 -Tag slow              # tags:["slow"] のみ
.\Physics\PhysicsView\run_physics_scenarios.ps1 -List                  # 実行せず一覧（タグ付き）

# 単一シナリオ
.\build\windows-debug\Physics\PhysicsView.exe --run-scenario Physics\PhysicsView\scenarios\10_fluid_dfsph_pool_settle.json
```

**命名規約:** `NN_<domain>_<name>.json`（`NN` は実行順を兼ねる番号帯、フラット構成）。

| 番号帯 | ドメイン | 番号帯 | ドメイン |
|---|---|---|---|
| 00–09 | `smoke` — 起動・Reset・ライフサイクル | 60–69 | `bound`/`emit` — メッシュ境界・エミッタ |
| 10–19 | `fluid` — SPH ソルバー単体 | 70–79 | `pipe` — Volume/Mesh 変換パイプライン |
| 20–29 | `rigid` — 剛体単体 | 80–89 | `capture` — スクリーンショット |
| 30–44 | `soft` — 軟体単体・軟体間/自己衝突 | 90–99 | `neg` — 異常系 |
| 50–59 | `couple` — Rigid↔Fluid / Soft↔Fluid | | |

現在 53 本（`04_smoke_new_scene`（File > New / `NewScene` コマンド）、`81_render_background` / `82_render_rigid_shaded` / `83_render_soft_shaded` / `84_render_shadows` / `85_render_ssfr_scene`（glTF レンダリング Phase 1–5）を含む）。全シナリオが「事前条件・変化（`store_as`+`post_assert` または初期値を含まない `expect_range`/`expect_not`）・不変条件」の三点契約で構成されている（詳細は
シナリオテストガイドの「アサーション三点契約」節）。

**タグ:** JSON トップレベルの `"tags": [...]` を `run_physics_scenarios.ps1` が読み、`-Tag`/`-ExcludeTag` で絞り込む
（C++ 側 `ScenarioRunner` は未知キーとして無視するのでシミュレーション自体には影響しない）。既定実行は
`known-fail` を除外する（`-ExcludeTag` の既定値）。

- `known-fail`: 2026-08-14 時点で該当シナリオは無し（`00`/`14`/`26`/`80`/`90`/`51`/`52`/`54` はいずれも
  実バグ修正済みで既定スイートに復帰している。詳細は 内部設計メモ 1 節の git 履歴参照）。
  タグ自体はテストを消さずバグの再現手順として保持する運用方針として残す
  （内部設計メモ 4.1）。
  **結合シナリオのプールは必ず剛体の落下カラム上に粒子が乗るシード間隔にすること**——`50`/`54` は
  半径 1.0（間隔 2.0）のプールが原点中心 1×1×1 の箱を完全に避けており、One-Way の SDF ペナルティが
  恒等的にゼロになっていた（`RigidBoundary::getBoundaryForce()` は SDF < 0 の粒子にしか作用しない）。
  また `GetMaxParticlePositionY` は箱が触れない縁の粒子で決まるため結合の有無を検出できない。
  結合の効き具合は `GetMaxParticleSpeed` で見ること。
- `slow`: 600 step 級の重いシナリオ（`17_fluid_pbsph_small_scale_regression` のみ）。日常のイテレーションで
  除外したい場合は `-ExcludeTag slow`。

**コマンド網羅チェック:** `CommandDispatcher`/`RigidBodyCommandDispatcher`/`SoftBodyCommandDispatcher` の
全コマンドが最低 1 本のシナリオから叩かれているかは、各ディスパッチャの `route()` 内のコマンド名リテラル
（`grep -oE '"[A-Za-z]+:?' *CommandDispatcher.cpp`）と `scenarios/*.json` 内の `"command"` 値の差分を取れば
機械的に確認できる。

### SPH ショーケースシナリオ（`scenarios/showcase/`）

`SHOWCASE_DAM_BREAK`/`SHOWCASE_SHEET_IMPACT`/`SHOWCASE_WATER_SPHERE`（各 `_PREVIEW` 版あり）の
大規模 SPH デモを PhysicsView から直接回して PLY 連番として書き出すための 6 本の JSON シナリオを
`Physics/PhysicsView/scenarios/showcase/` に置いてある（PLY 出力のみが目的——VDB 化・メッシュ化・
レンダリングは対象外）。

```powershell
.\build\windows-debug\Physics\PhysicsView.exe --run-scenario Physics\PhysicsView\scenarios\showcase\showcase_dam_break.json
```

`scenarios/showcase/` はサブディレクトリのため `run_physics_scenarios.ps1` の既定スイープ
（`scenarios/*.json` を非再帰 glob）には含まれない——1 本が数百〜数千物理ステップ・最大 80 万粒子の
本番ベイク（数分〜数十分）であり、日常の回帰テストが要求する秒オーダーとは性質が違うため、意図的に
対象外にしている。

**追加したコマンド**（`CommandDispatcher.h` のクラス doc コメントに詳細）:

| コマンド | 用途 |
|---|---|
| `AddBoundarySphere:cx,cy,cz,radius,maxPenetration` / `ClearBoundarySpheres` / `GetBoundarySphereCount` | `FluidWorld::addBoundarySphere()`（`SphereBoundary`、WCSPH/DFSPH で有効） |
| `AddFluidSourceBox:xmin,ymin,zmin,xmax,ymax,zmax` / `AddFluidSourceSphere:cx,cy,cz,radius` / `ClearFluidSources` / `GetFluidSourceRegionCount` | `FluidWorld::addFluidSourceBox()`/`addFluidSourceSphere()` ——複数の初期シード領域を同時に登録（例: シート+水塊5個）。単一ボックスの `SetFluidBounds` では表現できない構成に対応 |
| `SetFluidMaxParticles:<n>` | `FluidWorld::Params::maxParticles`（既定 50000、`*Fluid` 自身の既定と同じ——エミッター駆動のシーンだけ引き上げが要る） |
| `SetPLYOutputDir:<dir>` / `SavePLY:<path>` / `StepFrameAndSavePLY:<substeps>` | `FluidPLYWriter.h`（位置のみの binary PLY）。`StepFrameAndSavePLY` は物理ステップ N 回＋自動採番 PLY 書き出しを1コマンドにまとめたもの——`"repeat": frame_end` の1行でベイク全体を表現できる |
| `SetFluidBoundaryDamping:<ratio>` | `FluidWorld::Params::boundaryDampingRatio` → `ISPHSolver::setBoundaryDampingRatio()`（下記「境界の反発」節。2026-08-26 追加） |
| `LoadRenderBackground:<path>` / `ClearRenderBackground` / `SetRenderBackgroundTransform:px,py,pz,rx,ry,rz,s` | `RenderBackground`（`bgGltfRenderer_` = `Phantom::Gltf::GltfSceneRenderer` 1 個）に glTF/GLB/OBJ/STL の背景セットを読み込む／解放／ワールド変換。fluid と同一カメラ（`FluidRenderer` が唯一のソース）。`docs/todo/PLAN_physicsview_gltf_rendering.md` Phase 1、2026-09-08 |
| `SetEnvironment:<dir>` / `ClearRenderEnvironment` | `<dir>/{right,left,top,bottom,front,back}.png` を cubemap として `SSFluidRenderer::loadEnvMap()`（反射 + SSFR モード skybox）へ配布。glTF 側 skybox/実 IBL は Phase 6 送り |
| `SetLight:dx,dy,dz,r,g,b,intensity` | 共有 directional light（glTF パス + SSFR。7 float、退化方向/負値は `Error:`） |
| `SetRenderUseIBL:{0\|1}` / `GetRenderSceneState` | IBL フラグ（実 precompute は Phase 6 の TODO）／背景パス・primitive 数・環境・transform・ライトを JSON で返す |
| `SetRigidRenderMode:{wire\|shaded\|both}` / `GetRigidRenderMode` | 剛体の表示切替。`wire`（既定、`RigidBodyWireRenderer`）／`shaded`（`GltfBodyRenderer` = body ごとに `GltfSceneRenderer` + 合成した単位 Sphere/Box をPBR描画、共有カメラ＋directional light）／`both`。Plane shape（床）は shaded instance を作らない（背景ステージと冗長・z-fight）。glTF レンダリング Phase 2、2026-09-08 |
| `SetSoftRenderMode:{wire\|shaded\|both}` / `GetSoftRenderMode` | 軟体の表示切替。`shaded` は `GltfSoftRenderer`（faces を持つ body ごとに `GltfSceneRenderer`、`SoftMesh::particles`＋スムーズ法線を毎フレーム `updateMorphedGeometry()` でストリーム、`VK_CULL_MODE_NONE`＋シェーダで法線をカメラ方向反転して両面ライティング）。Rope は faces 無し → shaded 対象外。glTF レンダリング Phase 3、2026-09-08 |
| `SetShadowEnabled:{0\|1}` / `GetShadowEnabled` | 共有 directional light の shadow map（`FluidApp` が `Phantom::Gltf::ShadowMapPass` を 1 個所有、`onPreRender()` で depth-only pass、背景/剛体/軟体の glTF PBR パスがサンプルする）。off でも pass 自体は走り caster を描かないだけ（クリア済み depth = 遮蔽なし）。glTF レンダリング Phase 4、2026-09-08 |
| `SetSSFREnabled:{true\|false}` / `IsSSFREnabled` / `SetSSFRMode:<0..5>` | 画面空間流体レンダリング（`SSFRPanel` の enabled / mode インデックスを駆動、GUI と同期）。off でも `FluidApp` の linear-HDR オフスクリーン（`hdrScene_`）へ全不透明パスを描き、SSFR コンポジットが唯一の最終 pass として ACES + exposure を 1 回だけ適用（`mode -1` = 純パススルー）。on で流体サーフェス再構成 pre-pass が走り、glTF/剛体の depth を参照して遮蔽・屈折する。glTF レンダリング Phase 5、2026-09-08 |

**既知の簡略化**（各シナリオ JSON の `_comment` にも記載）:
- `showcase_dam_break.json`/`showcase_preview.json`: 崩れた水柱を裂く柱2本・段差1個の障害物は**入れていない**。
  これらは `is_static=true` の SPH 粒子として同一流体に混ぜる手法でしか表現できず、
  「1流体=1粒子集合」という native 側のモデルに対応する仕組みが無い。ダムブレイクの崩壊そのもの
  （このシーンの本質的な物理）はそのまま再現されている。
- `showcase_water_sphere.json`/`_preview.json`: 内部設計メモ
  の不安定バグは **2 段階で修正済み**—— `addBoundaryDensity()` の密度クランプ（同 9 節）と
  本番ティアの粘性（同 11 節）。**後者が無いと本番ティアは直らない**
  （9 節の修正は下見ティアでしか計測されていなかった）。両シナリオとも
  `SetFluidBoundaryDamping:0.35` を入れてあるが、これは**小さな寄与**にすぎない（同 11.5 節）。
  `emit_speed_jitter` は `AddEmitter` コマンドの引数に無いため native 既定の 0.1 のまま。

**元シーンの Z-up → PhysicsView（Y-up）の座標変換**: 全プリセットとも重力が水平2軸に成分を持たないため、
単純な軸入れ替え `(Bx, By, Bz) → (Bx, Bz, By)` で足りる（`gravity (0,0,-9.8)` → `(0,-9.8,0)`、
`FluidWorld::Params::gravity` の既定値と一致）。密度・圧力係数・粘性・タイムステップ等のスカラー値は
単位変換を挟まずそのまま使える。

## Architecture

### Phantom::Physics（`Physics/Physics/`）— コアライブラリ

**流体（SPH）**
- `ISPHSolver` — DFSPH/PBSPH/WCSPH の共通インターフェース。`simulate(dt, maxIter)` に加え、剛体境界（One-Way SDF: `addRigidBoundary`／Two-Way Akinci 境界粒子: `addRigidBoundaryParticles`）と SoftBody 境界粒子（`addSoftBoundaryParticles`）の登録口を持つ。`setMaxSubstep()` は DFSPH の適応サブステップ上限を明示する別名で、`getLastSolveStats()` は直前フレームの進行時間・サブステップ数・反復回数・収束／設定妥当性を返す。粒子数・位置・速度・密度・カーネル・基準密度も共通 getter で取得でき、PhysicsView の CPU ソルバー表示／WhiteWater 経路は具象流体へ分岐しない。Two-Way 系は DFSPH/PBSPH/WCSPH が実装（`supportsTwoWayCoupling()`）。（旧 MVCSolver は意図的に対応せず no-op を継承していたが、開発一時停止のため現在はビルド対象外 — 下記参照。）
- 各流体: `DFSPHSolver`/`PBSPHSolver`/`WCSPHSolver` + 対応する `*Fluid`/`*Particle` 型。DFSPH/PBSPH/WCSPH はいずれも `*Fluid::getKernel()` で自身の `SPHKernel` を公開する（ソルバー側がローカルに作り直すことはしない）。
- 境界表現: `RigidBoundary`（SDF ペナルティ）、`RigidBoundaryParticles`／`SoftBoundaryParticles`（Akinci 境界粒子、共に `IBoundaryParticles` 実装）、`SVBoundary`（Sparse Volume ベース）、`OctreeBoundary`（三角形メッシュ + オクトツリー）、`DMBoundary`（密度マップベース）。
- **ドメイン容器の解析境界（`ISPHSolver::setBoundary*()`）**: `PlaneBoundary`（無限半空間＝箱の内側）、`SphereBoundary`（閉じた球容器、内側が有効）、`PlateBoundary`（**有限平面＝薄い OBB、外側が有効**、内部設計メモ）。いずれも `IShapeBoundary` と `getSignedDistance() >= 0` の共通規約を実装する。`IShapeBoundary::sample()` は active 判定・符号付き距離・密度用距離・被覆率を `ShapeBoundarySample` として一度に返し、WCSPH/DFSPH/PBSPH の境界評価が同じ値の組を使うことを保証する（既存・独自境界は primitive virtual の既定合成実装で互換）。汎用の `setShapeBoundaries()`／`addShapeBoundary()` と、型別登録を含めて全消去する `clearShapeBoundaries()` も WCSPH/DFSPH/PBSPH が実装する。WCSPH は解析的な壁密度とペナルティ力を加え、DFSPH は境界密度と対応する α 勾配を必ず対で加えたうえで適応サブステップ用ペナルティ力を適用する。**WCSPH/DFSPH/PBSPH のいずれも Plane/Sphere/Plate/generic をサポートする**（2026-09、PBSPH インタフェース整合計画）。PBSPH は `PBSPHSolver::addShapeBoundaryConstraint()` で全 shape 種（plane 含む）に対し境界密度と対応する制約勾配を必ず対で加え（擬似境界粒子の重みは流体粒子と同じ `getMass()`）、その上でペナルティ位置補正＋hard clamp＋lambda clamp を効かせる。
- **Two-Way 境界粒子の共通化**（`BoundaryParticle.h`/`IBoundaryParticles.h`）: `RigidBoundaryParticles`/`SoftBoundaryParticles` はどちらも `worldPos`/`psi`/`accumForce` を持つ共通の `BoundaryParticle` 要素を `IBoundaryParticles::particles()` で公開する。`RigidBoundaryParticles` 固有の局所座標（rest-pose local position）は `localPositions()`（`particles()` と並列な配列）で別途公開する。DFSPHSolver/PBSPHSolver/WCSPHSolver はいずれも rigid 用・soft 用で本体が同一の `addBoundaryParticleDensity()`/`addBoundaryParticlePressure()`（PBSPH のみ `addBoundaryParticleConstraintGradient()` も）を rigid リスト・soft リストそれぞれに対して呼ぶ形に統一済み（`rigidBoundaryParticles_`/`softBoundaryParticles_` は `clearRigidBoundaryParticles()`/`clearSoftBoundaryParticles()` を独立に保つため2本のまま、要素の型のみ `IBoundaryParticles*` に統一）。`addRigidBoundaryParticles()`/`addSoftBoundaryParticles()` 等の登録 API 名は変更していない（`RigidBoundaryParticles*`/`SoftBoundaryParticles*` から `IBoundaryParticles*` への暗黙アップキャストで委譲するだけ）。
- **Emitter（連続粒子生成、内部設計メモ）**: `WCSPHFluid`/`DFSPHFluid`/`PBSPHFluid` はいずれも `addEmitter()`/`getEmitters()`/`clearEmitters()`/`updateEmitters(dt)` を持つ（`FlameFluid::Emitter` の一般化）。共通データ構造・端数蓄積ロジック（`rate*dt` の accumulator）は `Emitter.h`（`struct Emitter` + `accumulateEmission()`/`nextDiskLatticeOffset()`）に切り出し、粒子生成自体（`createParticle()`/`addParticle()` のシグネチャが流体ごとに違う）は各 `*Fluid::updateEmitters()` に残す。呼び出し側は `simulate()` の直前に `updateEmitters(dt)` を呼ぶだけでよい（各ソルバーは毎ステップ近傍探索をゼロから作り直す設計のため、途中で増えた粒子もソルバー側の変更なしに自動的に扱われる）。**`Emitter::particleRadius` は必ずシーンの他の粒子と同じ半径に合わせること**——WCSPH/DFSPH/PBSPH はいずれも半径から SPH 質量を導出する（`WCSPHParticle::getMass()`/`PBSPHParticle::getMass()` 等）ため、半径が食い違うと密度・圧力の較正（内部設計メモ）が崩れてソルバーが発散する（実際に `Emitter::particleRadius` の初期値が既定のシーン半径 1.0 に対し 0.05 のままだったため DFSPH が発散した実例あり、`DFSPHSolverTest.EmittedParticlesAtSceneRadiusStayFiniteWhileFallingIntoExistingFluid` で回帰確認）。`Physics/PhysicsView` の `FluidWorld::addEmitter()` はこの値を `params().radius` に強制上書きしてから登録するため、`CommandDispatcher`/`ControlPanel` 経由では発生しない。
- **Outflow Region（流出領域による粒子削除、オプション機能）**: `WCSPHFluid`/`DFSPHFluid`/`PBSPHFluid` はいずれも `addOutflowRegion()`/`getOutflowRegions()`/`clearOutflowRegions()`/`removeOutflowParticles()` を持つ（Emitter の対極——生成ではなく削除）。データ構造は `OutflowRegion.h`（`struct OutflowRegion { Math::Box3df bounds; }`、AABB のみ）。`removeOutflowParticles()` は `bounds.contains(pos, 0.0f)` が真の粒子を各 `*ParticleSoA::swapAndPop()`（`FlameParticleSoA::swapAndPop()` と同じ swap-and-pop パターン、`WCSPHParticleSoA`/`DFSPHParticleSoA`/`PBSPHParticleSoA` に追加）で削除する（順序は保持されない）。登録領域が空なら no-op——**完全にオプトイン**で、既存シーンは `addOutflowRegion()` を呼ばない限り一切影響を受けない。`Physics/PhysicsView` の `FluidWorld::stepFluidOnly()` は `fluidSolver_->simulate()` の直後に `updateOutflow()`（アクティブな `*Fluid::removeOutflowParticles()` へディスパッチ）を呼ぶ。`CommandDispatcher` の `AddOutflowRegion:minX,minY,minZ,maxX,maxY,maxZ`/`ClearOutflowRegions`/`GetOutflowRegionCount` と `ControlPanel` の「Outflow Regions」セクションから利用できる。GPU_CSPH は非対応（CPU 側 `*Fluid` を持たないため、Emitter 同様 no-op）。

**剛体**
- `RigidBody`、衝突は `BroadPhase`（`Phantom::Space::BVH` ベース）→ `NarrowPhase` → `CollisionPair`（`ContactManifold`）。
- コライダー: `ICollisionShape`／`ISoftCollider` を実装する `SphereCollider`/`PlaneCollider`/`RigidBodyCollider`。
- `RigidBodySolver` が積分・拘束解決を担当。`SelfCollision`/`CrossBodyCollision` は複数剛体間の追加チェック。

**軟体（SoftBody、XPBD ベース）**
- `ISoftBody` を実装する `ClothBody`/`JellyBody`/`RopeBody`（それぞれ `*Params` で生成）。`SoftMesh` が頂点・拘束の実体。
- 拘束: `IConstraint` を実装する `DistanceConstraint`/`BendConstraint`/`VolumeConstraint`/`ShapeMatchingConstraint`/`PinConstraint`。
- `XPBDSolver` が拘束の反復解決、`SoftBodySolver` が積分・剛体コライダー登録（`addRigidBodyCollider()`）を担当。

**三者結合（Rigid ↔ Fluid ↔ SoftBody）**
- `RigidFluidSolver` — 剛体を流体境界としてバインド（One-Way/Two-Way）。`bind()` は `std::deque<RigidFluidBinding>` に保持するため、後続の `bind()` で以前の `&binding.boundary`/`&binding.particles` が無効化されることはない（`clearBindings()` されるまで有効）。
- `SoftFluidSolver` — SoftBody を Akinci 境界粒子として流体にバインド（`setSoftCouplingFluidInfo()` で毎ステップ psi 再計算）。`bind()` の保持コンテナは `RigidFluidSolver` と同じ理由で `std::deque`。
- `RigidSoftSolver` — 剛体と SoftBody を直接結合（位置拘束コライダー + Two-Way 時は SDF ペナルティ反力）。`bind()` の保持コンテナも同様に `std::deque`。
- `PhysicsSolver` — 上記 3 つの結合ソルバーと呼び出し元供給の `ISPHSolver` を単一 `step()`/`stepUnconditional()` にまとめるトップレベルオーケストレータ。`setFluidSolver(ISPHSolver*)` は非所有ポインタを受け取る（呼び出し側が生成・破棄する。`PhysicsSolver` が `unique_ptr` で所有していた旧 API は廃止済み）。上位の統合アプリのシーン管理はこれを直接使用する想定（詳細は `PhysicsSolver.h` のクラス doc コメントを参照）。

### PhysicsView（`Physics/PhysicsView/`）— スタンドアロン ImGui + Vulkan アプリ

`FluidApp : VkAppBase` 直下。`FluidWorld`が`Physics::PhysicsSolver`を1個所有し、`RigidBodyWorld`/`SoftBodyWorld`は
その`rigidSolver()`/`softSolver()`/`rigidFluidSolver()`/`softFluidSolver()`を参照するだけの薄いプリセット構築
ファサード（`PhysicsSolver.h` が想定するトップレベル利用者と同じ単一`PhysicsSolver`所有パターン）。ただし`PhysicsSolver::step()`/
`stepUnconditional()`/`setRunning()`自体は使わない — FluidControlPanel/RigidBodyControlPanel/SoftBodyControlPanel
がそれぞれ独立にPlay/Pause/Stepできる必要があり（`PhysicsSolver::setRunning()`はfluid/rigid/softを1つの
フラグに束ねてしまう）、`FluidWorld::step()`/`stepOnce()`が`physicsSolver_.rigidFluidSolver()`/
`softFluidSolver()`を直接オーケストレーションする（詳細は`FluidWorld.h`のクラス doc コメント参照）。
- **起動時は空シーン**（2026-09-09）: `FluidApp::newScene()` が `onInit()` の最後に走り、3D シーンを空（流体粒子 0・剛体 0・軟体 0・glTF 背景なし）にする。`RigidBodyWorld`/`SoftBodyWorld` のコンストラクタはもうプリセットを組まず（旧: `SphereDrop`／`ClothTwoPin`）、`FluidWorld` も `world_.reset()` を起動時に呼ばない。プリセットは `RigidBody`/`SoftBody` 各パネルの Preset コンボ・シナリオの `SetPreset:` コマンド・`ControlPanel` の Reset で初めて構築される（コンボの getter は body 0 のとき `-1` を返し、同じプリセットを選び直しても再構築できるようにしてある）。**File メニューに `New`**（`FluidApp::newScene()` 直結）／シナリオコマンド **`NewScene`（別名 `New`）** — 同じ空シーン化ルーチンを手動／スクリプトから叩く（`CommandDispatcher::setOnNewScene()`、シナリオ `04_smoke_new_scene`）。`newScene()` = `FluidWorld::newScene()`（ソルバー破棄→粒子 0、emitter/outflow/source/境界/coupling を全クリア。`params()` と `SimulationType` は保持）+ `RigidBodyWorld::clear()` + `SoftBodyWorld::clear()` + `RenderBackground::clearBackground()`。Scene Objects パネルの "Fluid" だけは常設エントリとして残る（"0 particles" 表示）。
- `FluidWorld`（`FluidWorld.h`）— SPH 流体 + 内包する `RigidBodyWorld`（`rigid()`）+ 任意の Rigid-Fluid 結合（`setCouplingEnabled()`）。旧 `PhysicsSceneWorld` はここに統合済み。
- `SoftBodyWorld` — クロス/ロープ/ゼリーのシーン。`setSoftCouplingEnabled()`でSoftBody-Fluid結合も可能（UI・シナリオコマンドの配線は`FluidWorld`/`FluidApp`側）。
- `FluidCommandDispatcher`/`RigidBodyCommandDispatcher`/`SoftBodyCommandDispatcher` — `IScenarioDispatcher` を実装するコマンド文字列ディスパッチャ（シナリオテストガイド参照）。`CommandDispatcher` の `AddEmitter:cx,cy,cz,radius,rate,dirX,dirY,dirZ,speed`/`ClearEmitters`/`GetEmitterCount` が `FluidWorld::addEmitter()`（上記 Emitter 節）を駆動する。`ControlPanel` にも同機能の ImGui セクション（"Emitters"）がある。シナリオ例: `scenarios/dfsph_emitter_faucet.json`。
- 描画: `FluidRenderer`（パーティクル直接描画）と `SSFluidRenderer`（`Physics/FluidRenderer/` の SSFR、下記）を切替可能。`RigidBodyWireRenderer`/`SoftBodyWireRenderer` はワイヤーフレーム表示。
- glTF レンダリング（`docs/todo/PLAN_physicsview_gltf_rendering.md`、Phase 0–5 実装済み 2026-09-08、Phase 6 は任意）: `bgGltfRenderer_`（`Phantom::Gltf::GltfSceneRenderer` 1 個）が不透明背景セットを流体と同一カメラ（`FluidRenderer` が唯一のソース、`syncBackgroundCamera()` が `inverse(view)[3]` を eye に）で PBR 描画する。ドキュメント・環境・共有 directional light は `RenderBackground.{h,cpp}` が所有し、`CommandDispatcher` の `LoadRenderBackground`/`SetEnvironment`/`SetLight`/… と `ControlPage::Rendering`（`RenderingPanel`、"glTF Rendering" ページ）が駆動する。gltf.{vert,frag} は `CGLib/GltfViewer/shaders` から `PhysicsView/shaders/` へコピー（SDR シェーダ内トーンマップ版、Universe と同一）。Flame ページ中は `setVisible(false)`。Phase 2–3（2026-09-08）で剛体・軟体の PBR 描画を追加: `GltfBodyRenderer`（`PrimitiveGltf.{h,cpp}` の単位 Sphere/Box、`SetRigidRenderMode`）と `GltfSoftRenderer`（`SoftMeshGltf.{h,cpp}` で `SoftMesh` から合成、毎フレーム `GltfSceneRenderer::updateMorphedGeometry()` で position+法線ストリーム、cull-none、`SetSoftRenderMode`、Rope は対象外）。どちらも Vulkan 非依存の合成部を PhysicsTest で単体テスト。CGLib 側は `GltfGpuMesh::setKeepCpuVertices()`/`updatePositionsAndNormals()`・`GltfSceneRenderer::setDynamic()`/`setCullMode()`/`updateMorphedGeometry()` を純追加（既定不変）。Phase 4（2026-09-08、CGLib 変更なし）で共有 directional light の shadow map（`FluidApp` が `Phantom::Gltf::ShadowMapPass` を 1 個所有、`onPreRender()` で depth-only pass、背景/剛体/軟体が影を落とす/受ける、`SetShadowEnabled`）を追加。Phase 5（2026-09-08、CGLib 変更なし）で linear-HDR オフスクリーン合成: 9 個のシーンサブレンダラーを `add()` せず `hdrRenderers_` に集め `onPreRender()` で `hdrScene_`（`VulkanOffscreen` RGBA16F+depth）へ手動描画、`ssfrRenderer_` だけが VkAppBase サブレンダラーとして残りそのコンポジットがスワップチェーンへの唯一の最終 pass（HDR scene サンプル → 流体合成 → ACES+exposure 1 回）。`gltf.frag` は linear 出力へ（PhysicsView ローカル差分は Phase 3 の法線反転 + これの 2 点）。`SetSSFREnabled`/`SetSSFRMode`。共有 `Physics/FluidRenderer` の `BilateralFilter` 多重呼び出しバグ（frame あたり 1 descriptor set / UBO しか無く bind 中更新でコマンドバッファ無効化）も `SSFRPassConfig::setsPerFrame` 追加で修正（FluidStudio にも効く、既定 1 で挙動不変）。
- `FlameWorld`/`FlameControlPanel`/`FlameRenderer` — 炎 SPH（旧 FlameView を統合、下記 Flame 節）。`ControlPage::Flame` を開いている間だけ描画する独立ドメイン。

### Fluid_GPU_Vk（`Physics/Fluid_GPU_Vk/`）

`CSPHSolverVk` — Vulkan Compute による GPU 版 CSPH ソルバー。`CSPHParticleBufferVk`/`CSPHGridBufferVk` が SSBO を管理、シェーダーソースは `Shader/*.comp`（`Shader/compile_shaders.bat` で glslc により事前に `.comp.spv` へコンパイルし、`VulkanSPVResolver.h` 経由でランタイムに読み込む——他の Vulkan アプリ/ライブラリと同じ方式）。以前は `CSPHShaderSourceVk.h` に埋め込んだ GLSL 文字列を libshaderc で実行時コンパイルしていたが、2026-08-19 にこの事前コンパイル方式へ変更し libshaderc への依存を撤廃した。`FluidWorld::SimulationType::GPU_CSPH` から利用される（`setVulkanContext()` を事前に呼ぶ必要あり）。

### FluidRenderer（`Physics/FluidRenderer/`）

Screen Space Fluid Rendering（SSFR）パイプライン。`ParticleDepthRenderer`（深度）→ `BilateralFilter`（平滑化）→ `SSThicknessRenderer`（厚み）→ `SSReflectionRenderer`/`SSRefractionRenderer`（反射・屈折）を `SSFluidRenderer` が束ね、`SSFROffscreenSet` でオフスクリーンターゲットを管理する。

### Flame（`Physics/Physics/Flame*` + `Physics/PhysicsView/Flame*`）— 炎 SPH（実験的・独立系統）

内部設計メモに基づく、燃焼するガスを表現する SPH ソルバー。
`WCSPHParticle`/`WCSPHFluid`/`WCSPHSolver` と同じ 3 分割構成・同じ近傍探索（`Space::CSRNeighborList`）/
カーネル（`SPHKernel`）を土台にしているが、**Rigid/SoftBody 結合（`ISPHSolver`）を一切実装しない独立系統**。
`PhysicsSolver`（Rigid↔Fluid↔SoftBody 三者結合）にも登録されない。2026-09-08 に旧
`Physics/FlameView/` スタンドアロンビューアを PhysicsView へ統合したが、この「結合しない独立系統」
という性質は変えていない（PhysicsView の中でも fluid/rigid/soft とは一切カップリングしない）。

- `FlameParticle`/`FlameFluid`/`FlameSolver`（`Physics/Physics/`）— コアシミュレーション。
  `FlameParticle` は position/velocity/force/density に加え `temperature`/`fuel`/`soot`/`age` を保持し、
  `react(dt)` で近傍不要の燃焼反応（燃料減衰・発熱・煤生成）を独立に更新する。表面張力/法線は実装しない
  （炎は自由に拡散してよいため）。`FlameSolver::simulate(dt)` は密度→圧力/粘性パスに加え、渦度閉じ込め
  （2 パス: `addVorticity`/`addVorticityGradient`）・Boussinesq 浮力（`FlameParticle::applyBuoyancy()`、
  温度ベースの実効密度差から計算。素朴な符号（`-coe*(rhoEff-rho0)*gravity`）は熱い粒子で下向きになる
  バグを生むため、`FlameSolver.cpp`/`FlameParticle.cpp` のコメント参照の上で符号を反転してある点に注意）・
  カールノイズ（`glm::perlin` ベース、速度に直接加算）を実装する。`FlameFluid::updateEmitters()`/
  `removeDead()` がエミッタ生成と寿命管理（`age > lifeMax` または燃え尽きて常温近傍まで冷えたら削除）を担う。
- **PhysicsView 統合**（`Physics/PhysicsView/`、2026-09-08）— 旧 `FlameView` の中身を PhysicsView に
  折り込んだ。`FlameWorld`（`FlameFluid`+`FlameSolver` を所有、旧 `FlameApp::setupInitialScene` と同一の
  初期シーン・同じ固定 1/60 ステップ・同じ決定的シード）、`FlameControlPanel`（`ControlPage::Flame`、
  旧 `FlameApp::onImGui` の移植）、`FlameRenderer`/`FlamePipeline`/`FlameSmokePipeline`/`FlamePBVRPipeline`
  （旧 FlameView から移動、`namespace FlameView` → `Phantom` へ改名）で構成される。`FluidApp` は fluid/
  rigid/soft と並ぶ 1 ドメインとして扱い、独立に Play/Pause/Step できる。**軽量統合**——`CommandDispatcher`
  への配線・シナリオコマンド・シナリオテストは追加していない（旧 FlameView 同様、手動確認のみ）。
  - シミュレーションはネイティブの ~3 unit スケールのまま動かし、`FlameControlPanel` の「Display
    Transform」（`FlameWorld::RenderParams::renderScale`/`renderOffset`、既定 12 / (20,4,20)）で
    PhysicsView 共有カメラ（中心 (20,20,20)）空間へ写す純粋な描画変換をかける。SPH 状態は旧 FlameView と
    バイト一致。
  - `flame_*.{vert,frag}` は `PhysicsView/shaders/` に移動済み。`FlameRenderer` の 3 パイプラインが
    `VulkanSPVResolver` 経由で他シェーダーと同じ `shaders/` から名前で読む。
  - Flame ページを開いている間は `FlameRenderer` のみ描画し、fluid/SSFR/rigid/soft の各レンダラーは
    `setEnabled(false)`（`RigidBodyWireRenderer`/`SoftBodyWireRenderer` に `setEnabled` を追加した）。
    fluid/rigid/soft のシミュレーション自体は他ページと同様バックグラウンドで進む。
- **非スコープ（意図的）**: `RigidBoundary`/`addRigidBoundary()` 等の Rigid/SoftBody 境界結合、GPU 化
  （`Fluid_GPU_Vk` 相当）、煙レイヤー分離。将来の拡張候補として 内部設計メモ の Phase 4 に記載。

## Key Conventions

- **例外禁止**: このリポジトリ全体の規約に従い、`throw`/`try`/`catch` は使わない。エラーは `bool`/`std::optional` で返す。
- **One-Way / Two-Way の呼称**: 剛体または SoftBody が流体に一方的に力を及ぼす（SDF ペナルティ）場合が **One-Way**、Akinci 境界粒子により双方向に力が伝わる場合が **Two-Way (Track B)**。コード・コメント中でこの呼称が統一して使われている。
- **非所有ポインタの寿命（Tier 1 / Tier 2）**: 内部設計メモ の方針Aに基づき、`Physics/Physics` の公開 API は 2 階層に分かれる。
  - **Tier 1（シミュレーション実体）**: `ISPHSolver` 実装、`*Fluid`、`RigidBody`、`ISoftBody`、`ICollisionShape`。**常に呼び出し側が生成・破棄する**。`bindRigidBody()`/`bindSoftBody()`/`addRigidBoundary()`/`setFluidSolver()`/`RigidBodySolver::addBody()`/`SoftBodySolver::addBody()` 等が受け取るポインタはすべて非所有で、呼び出し元が寿命管理する（`PhysicsSolver.h` の doc コメントに明記）。ソルバー側は個体の削除 API を持たず、`clear()`/`clearBodies()`/`clearBindings()` で登録を一括で空にするのみ。`RigidBody`/`ISoftBody` の所有権は常に呼び出し側（PhysicsView の各 World、テストのローカル変数など）が持つ。
  - **Tier 2（ライブラリ内部の実装オブジェクト）**: `XPBDSolver`、`IConstraint`、`RigidBodyCollider`、`SparseVolumef`、境界粒子集合（`RigidBoundary`/`RigidBoundaryParticles`/`SoftBoundaryParticles`）。ライブラリ側が所有してよいが、外部に生ポインタを登録させるものはアドレス安定なコンテナで保持する必要がある（`RigidFluidSolver`/`SoftFluidSolver`/`RigidSoftSolver` の `bindings_` が `std::vector` ではなく `std::deque` なのはこのため）。
- **DFSPH の境界結合は「分子と分母を必ずセットで」**: `DFSPHSolver::addBoundaryParticleDensity()`（密度＝制約の分子）を呼ぶ箇所では必ず `addBoundaryParticleAlpha()`（α＝分母）も呼ぶこと。片方だけだと、ソルバーが「自分では解消できない密度超過」に対して無限大に近い剛性を計算して流体を吹き飛ばす。PBSPH 側の対応物は `addBoundaryParticleConstraintGradient()`（元々セットで呼ばれている）。
- **`SPHKernel::getCubicSpline()` は 3D 規格化済み**（`W(0) = 8/(π h³)`、`∫W dV = 1`）で、`getCubicSplineGradient()` と整合する。2026-08-14 以前は正規化定数が `h⁶/4` 倍ずれており、密度（W 由来）と DFSPH の α／圧力補正（∇W 由来）がシーンスケールに依存して食い違っていた。この修正で密度の絶対値が変わったため、旧カーネル前提でチューニングされた `pressureCoe` は再調整が必要だった。
- **`WCSPHFluid::estimatePressureCoe()` は単純な線形式**（2026-08-17、内部設計メモ section 4 の重力・目標密度誤差比・rest_density に基づく物理由来の導出は廃止）: `pressureCoe = pressureCoeScale * effectLength`。`ControlPanel`/`CommandDispatcher`（`SetFluidPressureCoeScale:`）/`FluidWorld::Params::pressureCoeScale` の既定値 `1960.0f` は、旧導出式の既定（`gravity=9.8`, `maxDensityErrorRatio=0.01`）と同じ挙動になるよう選んだ値。
- **境界の反発（壁の密度寄与と減衰、内部設計メモ 9 節）**:
  ドメイン壁は流体に対して 2 通りに効く——`WCSPHSolver::addBoundaryDensity()` の**密度寄与**（壁の
  向こう側を静止密度の流体と見なす半空間積分。壁際の粒子が密度不足で圧力ゼロに張り付き、
  荷重で潰れるのを防ぐために必須）と、`PlaneBoundary`/`SphereBoundary`/`PlateBoundary::getBoundaryForce()` の
  **ペナルティ力**。両方に落とし穴がある。
  - 密度寄与は**「不足分」を埋めるだけ**にすること（`headroom = restDensity - p.getDensity()` で
    クランプ済み）。流体近傍だけで既に静止密度に達している粒子に無条件に足すと、
    流体が既に詰め寄っている空間を二重に数えることになり、圧力スパイクでその層が吹き飛ぶ。
  - ペナルティ力 `-d/dt²` は**減衰の無い保存ばね**で、反発係数が実測 1.03——落ち着いたプールでは
    上に載った水の重みが跳ね返りを食うので表面化しないが、**空の容器に噴流が当たる**ような
    「上に何も無い」状況では丸ごと飛沫になる。`setBoundaryDampingRatio(ζ)`（WCSPH/DFSPH、既定 0 ＝
    従来と完全に同一）で法線速度の減衰を入れられる。ζ は `[0, 0.5]` にクランプされる
    （ばね剛性が `1/dt²` 固定＝ ω·dt = 1 なので、0.5 で 1 ステップぶんの法線速度をちょうど
    打ち消しきり、それ以上は打ち消しすぎて跳ね返りが復活する）。実用値は 0.35 前後。
    PBSPH は予測位置を `clampPosition()` で直接押し戻す方式なので、この減衰は不要。
    **ただし ζ は「密度寄与が正しいこと」の代わりにはならない。** 水球ショーケースでは
    密度クランプと粘性を直したあと、粒子の壁へのめり込みが h の 5% しか無くなり、
    ζ を 0〜0.5 で振っても最大速度が 1% 未満しか動かなくなった（同 11.5 節）。
    ζ が効くのは「素の壁に高速の流体が当たり、その上に何も載っていない」場面に限られる。
- **近傍探索は `Phantom::Space::CSRNeighborList` に一本化**: DFSPH/PBSPH/WCSPH/Flame いずれも `neighbors.build(positions, effectLength)` の 1 行で近傍探索と CSR 化をまとめて行う（`CGLib/Space/Space/NeighborList.h`）。内部実装は `IndexedSortBasedSearcher`（グリッド ID ソート + 前方 13 セル走査）で、返る行は「自分自身を含まない・`effectLength` 未満に距離フィルタ済み・対称（j が i の行にあれば i も j の行にある）」。**各パスは必ずペアではなく粒子でループすること**——ペアで並列化すると 1 反復が両端の粒子に書き込むため、同じ粒子を共有する 2 ペアが別スレッドに載って非アトミックな `+=` を競合させ、実行ごとに結果が変わる（`Physics/PhysicsTest/FluidDeterministicTest.cpp` が回帰確認）。行を `std::vector<int>` ではなく 2 本のフラット配列で持つのは、毎ステップ作り直す前提で粒子ごとのヒープ確保が支配的コストになるため（内部設計メモ 4 節）。粒子側の受け口は `Space::NeighborIndexView`（`std::vector<int>` から暗黙変換できるので、テストが手書きの近傍リストを渡す形はそのまま使える）。
- **`viscosityCoe` は解像度に依存しない（＝粒子半径で振ってはいけない）**: `WCSPHParticle::solveViscosityForce()` は
  `viscosityCoe * Δv * ∇²W_visc * m_j` を力に足し `forwardTime()` が `力/ρ` にするので、教科書
  （Müller 2003 の `μ Σ m_j Δv/ρ_j ∇²W`）の `1/ρ_j` が抜けているぶん `viscosityCoe ≡ μ/ρ` ——
  **動粘性係数（m²/s）そのもの**である。SPH のラプラシアン近似を通すと加速度は `viscosityCoe · ∇²v` で
  `h` に依存しない（実測: 物理的に同一のシーンを解像度 2 倍で回しても、同じ値なら減衰は 5% 以内で一致）。
  `particle_radius`/`time_step` のように「下見ティアは本番の N 倍」と振ると、下見の実効粘性が本番の
  N 倍になり別の絵になる（内部設計メモ 11.2 節）。
  なお `WCSPHFluid::estimateViscosityCoe()` の `effectLength^1.5` は**この点で誤り**だが、
  本番コードからは呼ばれていない（呼ぶのは同名のテストのみ）。
- **乱数は決定的シードが既定（`Physics/Physics/RandomSeed.h`）**: `WCSPHFluid`/`DFSPHFluid`/`PBSPHFluid`
  （エミッターの `speedJitter`）・`FlameFluid`（噴出ディスク/火花/煙）・`WhiteWaterSystem`（飛沫/泡の寿命）の
  `std::mt19937` はすべて `kDefaultRandomSeed` から始まる。2026-08-27 以前は `std::random_device{}()` で
  seed されており、**エミッターを使うシーンは実行のたびに違う結果になっていた**——ショーケースのベイクが
  再現できず、パラメータの A/B 実測もノイズに埋もれていた（同 11.4 節）。意図的にばらつかせたい場合は
  各クラスの `setRandomSeed()` に明示的に渡すこと（`std::random_device{}()` を渡せば従来の挙動）。
  回帰確認は `FluidDeterministicTest.cpp` の `*EmitterJitterIsReproducible` /
  `WCSPHEmitterDrivenSceneRepeatedRunsMatch` / `EmitterRandomSeedSelectsTheDraw`。
- **新しい流体ソルバーを追加する場合**: `ISPHSolver` を実装し、Two-Way 結合をサポートしないなら `addRigidBoundaryParticles`/`addSoftBoundaryParticles`/`supportsTwoWayCoupling()` はデフォルト実装（no-op）のままでよい。サポートする場合は `IBoundaryParticles*` のリストを rigid 用・soft 用の2本持ち（`clearRigidBoundaryParticles()`/`clearSoftBoundaryParticles()` を独立に保つため）、両リストに対して同一の `addBoundaryParticleDensity()`/`addBoundaryParticlePressure()` を呼ぶ形にする（DFSPHSolver/PBSPHSolver/WCSPHSolver 参照）。解析境界は `setShapeBoundaries()` を主経路にし、Plane/Sphere/Plate/generic は WCSPH/DFSPH/PBSPH が実装する。DFSPH・PBSPH のような制約方式では境界密度と α／制約勾配 counterpart を必ず対で追加すること（PBSPH は `addShapeBoundaryConstraint()` で全 shape 種を1つのループにまとめている）。
- **`PhysicsSolver` の使い分けに注意**: 新しいアプリ/機能で Rigid/Fluid/SoftBody 結合が必要な場合は `Physics::PhysicsSolver` を使うのが標準。`PhysicsView`の`FluidWorld`/`RigidBodyWorld`/`SoftBodyWorld`も内部では単一の`PhysicsSolver`を共有する（上記参照）が、`PhysicsSolver::step()`/`setRunning()`は使わず独自に個別Play/Pauseをオーケストレーションしている点に注意（fluid/rigid/softを独立制御する必要がないアプリでは`PhysicsSolver::step()`/`stepUnconditional()`/`setRunning()`をそのまま使う方が単純）。
- **SPH の長さ単位はシーンごとの暗黙の取り決め（内部設計メモ 参照）**: `Physics/Physics` 内部のどの型も長さの単位を明示的には定義していない。本コードベースの慣例的な既定シーン（`radius=1` 等）は暗黙に「1 unit ≒ 1m」を仮定しており、重力の既定値（`DFSPHSolver`/`PBSPHSolver`/`WCSPHSolver` の `externalForce{0,-9.8f,0}`・`.fsscene` の `gravityY="-9.8"` 等）はこの前提のもとでの現実の重力加速度（9.8 m/s²）をそのまま採用している。**シーン全体の長さスケールを変える場合**（例: 粒子半径を 1.0 → 0.01 にして「1cm 粒子」を表現する）は、以下も比例して再スケールしないと同じ相対挙動にならない: 重力・`RigidBoundary::penaltyStiffness_`（Phase 2 以降は `estimateStiffness(dt)` で `dt` から自動導出可能）・`WCSPHFluid::pressureCoe`/`PBSPHFluid::stiffness`（Phase 1 以降は `estimatePressureCoe()`/`setPressureCoeFromScale()` で自動導出可能）・`maxTimeStep`/`boundaryTimeStep`（Phase 6 時点では手動比例のまま、既定値 `0.01f` は `radius=1` の CFL 見積もり相当）。DFSPH/PBSPH/WCSPH いずれも `setEffectLength()` を呼ばずに `simulate()` すると effectLength が `0.f` のまま（Phase 5）で no-op になる点にも注意。
