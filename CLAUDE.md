# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

流体・剛体・軟体（クロス／ゼリー／ロープ）シミュレーションと SPH ベースの三者結合（Rigid↔Fluid↔SoftBody）を提供するモジュール群。
`Physics`（コアライブラリ）、`PhysicsTest`（GoogleTest）、`PhysicsView`（スタンドアロン ImGui + Vulkan ビューア）、`Fluid_GPU_Vk`（GPU Compute CSPH）、`FluidRenderer`（Screen Space Fluid Rendering）、`FlameView`（炎 SPH の実験的スタンドアロンビューア、下記）の 6 プロジェクトで構成される。単独の `.sln` は持たず、すべて上位の `Phantom2026.sln` でビルドする。

親リポジトリの CLAUDE.md（`../CLAUDE.md`）にビルド方法・全体アーキテクチャ・命名規則が記載されているのであわせて参照すること。

## Build

CMake が唯一のビルド手段（2026-08-19、`.vcxproj` は全削除済み。詳細は
`docs/todo/PLAN_crossplatform_non_cgapp_build.md` Phase 5、親リポジトリ `../CLAUDE.md` の Build 節を参照）。

```powershell
# リポジトリルートから、Physics単体を設定・ビルド
cmake -S Physics -B Physics/build_windows -DCMAKE_BUILD_TYPE=Debug
cmake --build Physics/build_windows

# または、ルートの CMakePresets.json 経由でリポジトリ全体を一括ビルド
cmake --preset windows-debug
cmake --build --preset windows-debug
```

ターゲット: `PhysicsCore`, `PhysicsTest`, `PhysicsView`, `FlameView`, `Fluid_GPU_Vk`(`FluidGPUVkCore`),
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

`docs/todo/PLAN_physics_scenario_test_rebuild.md` に基づき全面再構築済み（2026-08）。ランナーは
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

現在 47 本。全シナリオが「事前条件・変化（`store_as`+`post_assert` または初期値を含まない `expect_range`/`expect_not`）・不変条件」の三点契約で構成されている（詳細は
`docs/guide/scenario_test_guide.md`「アサーション三点契約」節）。

**タグ:** JSON トップレベルの `"tags": [...]` を `run_physics_scenarios.ps1` が読み、`-Tag`/`-ExcludeTag` で絞り込む
（C++ 側 `ScenarioRunner` は未知キーとして無視するのでシミュレーション自体には影響しない）。既定実行は
`known-fail` を除外する（`-ExcludeTag` の既定値）。

- `known-fail`: 2026-08-14 時点で該当シナリオは無し（`00`/`14`/`26`/`80`/`90`/`51`/`52`/`54` はいずれも
  実バグ修正済みで既定スイートに復帰している。詳細は `docs/issue/CODEBASE_ISSUES.md` 1 節の git 履歴参照）。
  タグ自体はテストを消さずバグの再現手順として保持する運用方針として残す
  （`docs/todo/PLAN_physics_scenario_test_rebuild.md` 4.1）。
  MVC ソルバー（旧 `13_fluid_mvc_pool_settle.json`）は開発一時停止のためプロジェクトから除外し、
  `Physics/_paused_MVC/` に退避した（シナリオ・テスト共に同様、`known-fail` タグではなくビルド対象除外で扱う）。
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

### SPH ショーケースの native 移植（`scenarios/showcase/`、2026-08-26）

`CGApp/blender/PyFluid/crystal_sph_addon` のショーケースプリセット（`presets.py` の
`SHOWCASE_DAM_BREAK`/`SHOWCASE_SHEET_IMPACT`/`SHOWCASE_WATER_SPHERE`、各 `_PREVIEW` 版）は
Blender を経由しないと再現できなかった——シミュレーション自体は `Physics` ライブラリ内で完結する
（アドオン側は `crystal_fluid.pyd` 経由でこのライブラリを直接呼んでいるだけ）ため、Blender/Python
なしに同じ物理設定を PhysicsView から直接回して PLY 連番として書き出せるよう、6 本の JSON シナリオを
`Physics/PhysicsView/scenarios/showcase/` に追加した（PLY 出力のみが目的——VDB 化・メッシュ化・
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
| `AddBoundarySphere:cx,cy,cz,radius,maxPenetration` / `ClearBoundarySpheres` / `GetBoundarySphereCount` | `FluidWorld::addBoundarySphere()`（`SphereBoundary`、WCSPH のみ有効） |
| `AddFluidSourceBox:xmin,ymin,zmin,xmax,ymax,zmax` / `AddFluidSourceSphere:cx,cy,cz,radius` / `ClearFluidSources` / `GetFluidSourceRegionCount` | `FluidWorld::addFluidSourceBox()`/`addFluidSourceSphere()` ——複数の初期シード領域を同時に登録（例: シート+水塊5個）。単一ボックスの `SetFluidBounds` では表現できない構成に対応 |
| `SetFluidMaxParticles:<n>` | `FluidWorld::Params::maxParticles`（既定 50000、`*Fluid` 自身の既定と同じ——エミッター駆動のシーンだけ引き上げが要る） |
| `SetPLYOutputDir:<dir>` / `SavePLY:<path>` / `StepFrameAndSavePLY:<substeps>` | `FluidPLYWriter.h`（位置のみの binary PLY）。`StepFrameAndSavePLY` は物理ステップ N 回＋自動採番 PLY 書き出しを1コマンドにまとめたもの——`"repeat": frame_end` の1行でベイク全体を表現できる |
| `SetFluidBoundaryDamping:<ratio>` | `FluidWorld::Params::boundaryDampingRatio` → `ISPHSolver::setBoundaryDampingRatio()`（下記「境界の反発」節。2026-08-26 追加） |

**既知の簡略化**（各シナリオ JSON の `_comment` にも記載）:
- `showcase_dam_break.json`/`showcase_preview.json`: 崩れた水柱を裂く柱2本・段差1個の障害物は**移植していない**。
  アドオン側はこれを `is_static=true` の SPH 粒子として同一流体に混ぜる Python 側の手法で表現しており、
  native 側の「1流体=1粒子集合」というモデルに対応する仕組みが無い。ダムブレイクの崩壊そのもの
  （このシーンの本質的な物理）はそのまま移植されている。
- `showcase_water_sphere.json`/`_preview.json`: `docs/issue/water_sphere_showcase_emitter_instability.md`
  の不安定バグは **2 段階で修正済み**——2026-08-26 に `addBoundaryDensity()` の密度クランプ（同 9 節）、
  2026-08-27 に本番ティアの粘性（同 11 節）。**後者が無いと本番ティアは直らない**
  （9 節の修正は下見ティアでしか計測されていなかった）。両シナリオとも
  `SetFluidBoundaryDamping:0.35` を入れてあるが、これは**小さな寄与**にすぎない（同 11.5 節）。
  その調査で見つかったアドオン側だけの実バグ（エミッターが設計位置ではなくワールド原点に立つ）は
  ここでは踏まない（`AddEmitter` で設計位置を直接渡すため。アドオン側は 2026-08-26 修正済み）。
  `emit_speed_jitter`（アドオン既定 0.02）も `AddEmitter` コマンドの引数に無いため native 既定の 0.1 のまま。

**Blender（Z-up）→ PhysicsView（Y-up）の座標変換**: 全プリセットとも重力が水平2軸に成分を持たないため、
単純な軸入れ替え `(Bx, By, Bz) → (Bx, Bz, By)` で足りる（`gravity (0,0,-9.8)` → `(0,-9.8,0)`、
`FluidWorld::Params::gravity` の既定値と一致）。密度・圧力係数・粘性・タイムステップ等のスカラー値は
Blender 側もこの `Physics` ライブラリを直接呼んでいるため無変換でそのまま移植できる
（`sim_core.py` の `fluid.set_density(item.rest_density)` 等、単位変換を一切挟んでいない）。

## Architecture

### Phantom::Physics（`Physics/Physics/`）— コアライブラリ

**流体（SPH）**
- `ISPHSolver` — DFSPH/PBSPH/WCSPH の共通インターフェース。`simulate(dt, maxIter)` に加え、剛体境界（One-Way SDF: `addRigidBoundary`／Two-Way Akinci 境界粒子: `addRigidBoundaryParticles`）と SoftBody 境界粒子（`addSoftBoundaryParticles`）の登録口を持つ。Two-Way 系は DFSPH/PBSPH/WCSPH が実装（`supportsTwoWayCoupling()`）。（旧 MVCSolver は意図的に対応せず no-op を継承していたが、開発一時停止のため現在はビルド対象外 — 下記参照。）
- 各流体: `DFSPHSolver`/`PBSPHSolver`/`WCSPHSolver` + 対応する `*Fluid`/`*Particle` 型。DFSPH/PBSPH/WCSPH はいずれも `*Fluid::getKernel()` で自身の `SPHKernel` を公開する（ソルバー側がローカルに作り直すことはしない）。
- 境界表現: `RigidBoundary`（SDF ペナルティ）、`RigidBoundaryParticles`／`SoftBoundaryParticles`（Akinci 境界粒子、共に `IBoundaryParticles` 実装）、`SVBoundary`（Sparse Volume ベース）、`OctreeBoundary`（三角形メッシュ + オクトツリー）、`DMBoundary`（密度マップベース）。
- **ドメイン容器の解析境界（`ISPHSolver::setBoundary*()`）**: `PlaneBoundary`（無限半空間＝箱の内側）、`SphereBoundary`（閉じた球容器、内側が有効、WCSPH のみ）、`PlateBoundary`（**有限平面＝薄い OBB、外側が有効**、WCSPH のみ、`docs/todo/PLAN_sph_showcase_waterfall.md`）。いずれも `getSignedDistance() >= 0` が有効領域という共通規約で、`WCSPHSolver::addBoundaryForce()`（ペナルティ力）と `addBoundaryDensity()`（壁の向こうを静止密度流体と見なす半空間積分。`PlateBoundary` は縁で `getRimFraction()` により逓減）の両方に寄与する。`PlateBoundary` は零厚ではなく半厚を持つため、めり込み量が半厚で自動的に有界になり（`SphereBoundary` の `maxPenetration` に相当する安全弁が要らない）、両面から等しく弾く。DFSPH/PBSPH が `SphereBoundary`/`PlateBoundary` を実装しないのは、密度と α／制約勾配を必ずセットで足す必要があり密度だけの寄与を入れられないため（下記「DFSPH の境界結合は…」）。
- **Two-Way 境界粒子の共通化**（`BoundaryParticle.h`/`IBoundaryParticles.h`）: `RigidBoundaryParticles`/`SoftBoundaryParticles` はどちらも `worldPos`/`psi`/`accumForce` を持つ共通の `BoundaryParticle` 要素を `IBoundaryParticles::particles()` で公開する。`RigidBoundaryParticles` 固有の局所座標（rest-pose local position）は `localPositions()`（`particles()` と並列な配列）で別途公開する。DFSPHSolver/PBSPHSolver/WCSPHSolver はいずれも rigid 用・soft 用で本体が同一の `addBoundaryParticleDensity()`/`addBoundaryParticlePressure()`（PBSPH のみ `addBoundaryParticleConstraintGradient()` も）を rigid リスト・soft リストそれぞれに対して呼ぶ形に統一済み（`rigidBoundaryParticles_`/`softBoundaryParticles_` は `clearRigidBoundaryParticles()`/`clearSoftBoundaryParticles()` を独立に保つため2本のまま、要素の型のみ `IBoundaryParticles*` に統一）。`addRigidBoundaryParticles()`/`addSoftBoundaryParticles()` 等の登録 API 名は変更していない（`RigidBoundaryParticles*`/`SoftBoundaryParticles*` から `IBoundaryParticles*` への暗黙アップキャストで委譲するだけ）。
- **Emitter（連続粒子生成、`docs/todo/PLAN_physics_fluid_emitter.md`）**: `WCSPHFluid`/`DFSPHFluid`/`PBSPHFluid` はいずれも `addEmitter()`/`getEmitters()`/`clearEmitters()`/`updateEmitters(dt)` を持つ（`FlameFluid::Emitter` の一般化）。共通データ構造・端数蓄積ロジック（`rate*dt` の accumulator）は `Emitter.h`（`struct Emitter` + `accumulateEmission()`/`nextDiskLatticeOffset()`）に切り出し、粒子生成自体（`createParticle()`/`addParticle()` のシグネチャが流体ごとに違う）は各 `*Fluid::updateEmitters()` に残す。呼び出し側は `simulate()` の直前に `updateEmitters(dt)` を呼ぶだけでよい（各ソルバーは毎ステップ近傍探索をゼロから作り直す設計のため、途中で増えた粒子もソルバー側の変更なしに自動的に扱われる）。**`Emitter::particleRadius` は必ずシーンの他の粒子と同じ半径に合わせること**——WCSPH/DFSPH/PBSPH はいずれも半径から SPH 質量を導出する（`WCSPHParticle::getMass()`/`PBSPHParticle::getMass()` 等）ため、半径が食い違うと密度・圧力の較正（`docs/todo/PLAN_sph_scale_invariance.md`）が崩れてソルバーが発散する（実際に `Emitter::particleRadius` の初期値が既定のシーン半径 1.0 に対し 0.05 のままだったため DFSPH が発散した実例あり、`DFSPHSolverTest.EmittedParticlesAtSceneRadiusStayFiniteWhileFallingIntoExistingFluid` で回帰確認）。`Physics/PhysicsView` の `FluidWorld::addEmitter()` はこの値を `params().radius` に強制上書きしてから登録するため、`CommandDispatcher`/`ControlPanel` 経由では発生しない。
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
- `PhysicsSolver` — 上記 3 つの結合ソルバーと呼び出し元供給の `ISPHSolver` を単一 `step()`/`stepUnconditional()` にまとめるトップレベルオーケストレータ。`setFluidSolver(ISPHSolver*)` は非所有ポインタを受け取る（呼び出し側が生成・破棄する。`PhysicsSolver` が `unique_ptr` で所有していた旧 API は廃止済み）。**`CGApp/Universe` はこれを直接使用する**（詳細は `PhysicsSolver.h` のクラス doc コメントを参照）。

### PhysicsView（`Physics/PhysicsView/`）— スタンドアロン ImGui + Vulkan アプリ

`FluidApp : VkAppBase` 直下。`FluidWorld`が`Physics::PhysicsSolver`を1個所有し、`RigidBodyWorld`/`SoftBodyWorld`は
その`rigidSolver()`/`softSolver()`/`rigidFluidSolver()`/`softFluidSolver()`を参照するだけの薄いプリセット構築
ファサード（Universeの`UniverseScene`と同じ単一`PhysicsSolver`所有パターン）。ただし`PhysicsSolver::step()`/
`stepUnconditional()`/`setRunning()`自体は使わない — FluidControlPanel/RigidBodyControlPanel/SoftBodyControlPanel
がそれぞれ独立にPlay/Pause/Stepできる必要があり（`PhysicsSolver::setRunning()`はfluid/rigid/softを1つの
フラグに束ねてしまう）、`FluidWorld::step()`/`stepOnce()`が`physicsSolver_.rigidFluidSolver()`/
`softFluidSolver()`を直接オーケストレーションする（詳細は`FluidWorld.h`のクラス doc コメント参照）。
- `FluidWorld`（`FluidWorld.h`）— SPH 流体 + 内包する `RigidBodyWorld`（`rigid()`）+ 任意の Rigid-Fluid 結合（`setCouplingEnabled()`）。旧 `PhysicsSceneWorld` はここに統合済み。
- `SoftBodyWorld` — クロス/ロープ/ゼリーのシーン。`setSoftCouplingEnabled()`でSoftBody-Fluid結合も可能（UI・シナリオコマンドの配線は`FluidWorld`/`FluidApp`側）。
- `FluidCommandDispatcher`/`RigidBodyCommandDispatcher`/`SoftBodyCommandDispatcher` — `IScenarioDispatcher` を実装するコマンド文字列ディスパッチャ（`docs/guide/scenario_test_guide.md` 参照）。`CommandDispatcher` の `AddEmitter:cx,cy,cz,radius,rate,dirX,dirY,dirZ,speed`/`ClearEmitters`/`GetEmitterCount` が `FluidWorld::addEmitter()`（上記 Emitter 節）を駆動する。`ControlPanel` にも同機能の ImGui セクション（"Emitters"）がある。シナリオ例: `scenarios/dfsph_emitter_faucet.json`。
- 描画: `FluidRenderer`（パーティクル直接描画）と `SSFluidRenderer`（`Physics/FluidRenderer/` の SSFR、下記）を切替可能。`RigidBodyWireRenderer`/`SoftBodyWireRenderer` はワイヤーフレーム表示。

### Fluid_GPU_Vk（`Physics/Fluid_GPU_Vk/`）

`CSPHSolverVk` — Vulkan Compute による GPU 版 CSPH ソルバー。`CSPHParticleBufferVk`/`CSPHGridBufferVk` が SSBO を管理、シェーダーソースは `Shader/*.comp`（`Shader/compile_shaders.bat` で glslc により事前に `.comp.spv` へコンパイルし、`VulkanSPVResolver.h` 経由でランタイムに読み込む——他の Vulkan アプリ/ライブラリと同じ方式）。以前は `CSPHShaderSourceVk.h` に埋め込んだ GLSL 文字列を libshaderc で実行時コンパイルしていたが、2026-08-19 にこの事前コンパイル方式へ変更し libshaderc への依存を撤廃した。`FluidWorld::SimulationType::GPU_CSPH` から利用される（`setVulkanContext()` を事前に呼ぶ必要あり）。

### FluidRenderer（`Physics/FluidRenderer/`）

Screen Space Fluid Rendering（SSFR）パイプライン。`ParticleDepthRenderer`（深度）→ `BilateralFilter`（平滑化）→ `SSThicknessRenderer`（厚み）→ `SSReflectionRenderer`/`SSRefractionRenderer`（反射・屈折）を `SSFluidRenderer` が束ね、`SSFROffscreenSet` でオフスクリーンターゲットを管理する。

### Flame（`Physics/Physics/Flame*` + `Physics/FlameView/`）— 炎 SPH（実験的・独立系統）

`docs/idea/sph_flame.txt`／`docs/todo/sph_flame_plan.md` に基づく、燃焼するガスを表現する SPH ソルバー。
`WCSPHParticle`/`WCSPHFluid`/`WCSPHSolver` と同じ 3 分割構成・同じ近傍探索（`Space::CSRNeighborList`）/
カーネル（`SPHKernel`）を土台にしているが、**Rigid/SoftBody 結合（`ISPHSolver`）を一切実装しない独立系統**。
`PhysicsSolver`（Rigid↔Fluid↔SoftBody 三者結合）にも登録されない。

- `FlameParticle`/`FlameFluid`/`FlameSolver`（`Physics/Physics/`）— コアシミュレーション。
  `FlameParticle` は position/velocity/force/density に加え `temperature`/`fuel`/`soot`/`age` を保持し、
  `react(dt)` で近傍不要の燃焼反応（燃料減衰・発熱・煤生成）を独立に更新する。表面張力/法線は実装しない
  （炎は自由に拡散してよいため）。`FlameSolver::simulate(dt)` は密度→圧力/粘性パスに加え、渦度閉じ込め
  （2 パス: `addVorticity`/`addVorticityGradient`）・Boussinesq 浮力（`FlameParticle::applyBuoyancy()`、
  温度ベースの実効密度差から計算。素朴な符号（`-coe*(rhoEff-rho0)*gravity`）は熱い粒子で下向きになる
  バグを生むため、`FlameSolver.cpp`/`FlameParticle.cpp` のコメント参照の上で符号を反転してある点に注意）・
  カールノイズ（`glm::perlin` ベース、速度に直接加算）を実装する。`FlameFluid::updateEmitters()`/
  `removeDead()` がエミッタ生成と寿命管理（`age > lifeMax` または燃え尽きて常温近傍まで冷えたら削除）を担う。
- `FlameView`（`Physics/FlameView/`）— 最小限のスタンドアロン ImGui + Vulkan ビューア。`FlameApp : VkAppBase`
  が `FlameFluid`/`FlameSolver` のみを所有（Rigid/SoftBody 結合口に触れようがない構成）。`FlameRenderer`/
  `FlamePipeline` は `PhysicsView/FluidRenderer`/`FluidPipeline` とは別実装の、position+temperature の
  2 頂点属性のみを持つ加算合成ポイントスプライトパイプライン（`shaders/flame_point.vert/.frag` で温度→
  黒体放射風グラデーションを簡易近似）。シナリオテスト自動化は無し（実験用途、手動確認のみ）。
- **非スコープ（意図的）**: `RigidBoundary`/`addRigidBoundary()` 等の Rigid/SoftBody 境界結合、GPU 化
  （`Fluid_GPU_Vk` 相当）、煙レイヤー分離。将来の拡張候補として `docs/todo/sph_flame_plan.md` の Phase 4 に記載。

## Key Conventions

- **例外禁止**: このリポジトリ全体の規約（`docs/guide/conventions.md`）に従い、`throw`/`try`/`catch` は使わない。エラーは `bool`/`std::optional` で返す。
- **One-Way / Two-Way の呼称**: 剛体または SoftBody が流体に一方的に力を及ぼす（SDF ペナルティ）場合が **One-Way**、Akinci 境界粒子により双方向に力が伝わる場合が **Two-Way (Track B)**。コード・コメント中でこの呼称が統一して使われている。
- **非所有ポインタの寿命（Tier 1 / Tier 2）**: `docs/todo/PLAN_physics_ownership_and_coupling_unification.md` の方針Aに基づき、`Physics/Physics` の公開 API は 2 階層に分かれる。
  - **Tier 1（シミュレーション実体）**: `ISPHSolver` 実装、`*Fluid`、`RigidBody`、`ISoftBody`、`ICollisionShape`。**常に呼び出し側が生成・破棄する**。`bindRigidBody()`/`bindSoftBody()`/`addRigidBoundary()`/`setFluidSolver()`/`RigidBodySolver::addBody()`/`SoftBodySolver::addBody()` 等が受け取るポインタはすべて非所有で、呼び出し元が寿命管理する（`PhysicsSolver.h` の doc コメントに明記）。ソルバー側は個体の削除 API を持たず、`clear()`/`clearBodies()`/`clearBindings()` で登録を一括で空にするのみ。`RigidBody`/`ISoftBody` の所有権は常に呼び出し側（Universe の各 Component、PhysicsView の各 World、テストのローカル変数など）が持つ。
  - **Tier 2（ライブラリ内部の実装オブジェクト）**: `XPBDSolver`、`IConstraint`、`RigidBodyCollider`、`SparseVolumef`、境界粒子集合（`RigidBoundary`/`RigidBoundaryParticles`/`SoftBoundaryParticles`）。ライブラリ側が所有してよいが、外部に生ポインタを登録させるものはアドレス安定なコンテナで保持する必要がある（`RigidFluidSolver`/`SoftFluidSolver`/`RigidSoftSolver` の `bindings_` が `std::vector` ではなく `std::deque` なのはこのため）。
- **DFSPH の境界結合は「分子と分母を必ずセットで」**: `DFSPHSolver::addBoundaryParticleDensity()`（密度＝制約の分子）を呼ぶ箇所では必ず `addBoundaryParticleAlpha()`（α＝分母）も呼ぶこと。片方だけだと、ソルバーが「自分では解消できない密度超過」に対して無限大に近い剛性を計算して流体を吹き飛ばす。PBSPH 側の対応物は `addBoundaryParticleConstraintGradient()`（元々セットで呼ばれている）。
- **`SPHKernel::getCubicSpline()` は 3D 規格化済み**（`W(0) = 8/(π h³)`、`∫W dV = 1`）で、`getCubicSplineGradient()` と整合する。2026-08-14 以前は正規化定数が `h⁶/4` 倍ずれており、密度（W 由来）と DFSPH の α／圧力補正（∇W 由来）がシーンスケールに依存して食い違っていた。この修正で密度の絶対値が変わったため、旧カーネル前提でチューニングされた `pressureCoe` は再調整が必要だった。
- **`WCSPHFluid::estimatePressureCoe()` は単純な線形式**（2026-08-17、`docs/todo/PLAN_sph_scale_invariance.md` section 4 の重力・目標密度誤差比・rest_density に基づく物理由来の導出は廃止）: `pressureCoe = pressureCoeScale * effectLength`。`ControlPanel`/`CommandDispatcher`（`SetFluidPressureCoeScale:`）/`FluidWorld::Params::pressureCoeScale` の既定値 `1960.0f` は、旧導出式の既定（`gravity=9.8`, `maxDensityErrorRatio=0.01`）と同じ挙動になるよう選んだ値。
- **境界の反発（壁の密度寄与と減衰、`docs/issue/water_sphere_showcase_emitter_instability.md` 9 節）**:
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
- **近傍探索は `Phantom::Space::CSRNeighborList` に一本化**: DFSPH/PBSPH/WCSPH/Flame いずれも `neighbors.build(positions, effectLength)` の 1 行で近傍探索と CSR 化をまとめて行う（`CGLib/Space/Space/NeighborList.h`）。内部実装は `IndexedSortBasedSearcher`（グリッド ID ソート + 前方 13 セル走査）で、返る行は「自分自身を含まない・`effectLength` 未満に距離フィルタ済み・対称（j が i の行にあれば i も j の行にある）」。**各パスは必ずペアではなく粒子でループすること**——ペアで並列化すると 1 反復が両端の粒子に書き込むため、同じ粒子を共有する 2 ペアが別スレッドに載って非アトミックな `+=` を競合させ、実行ごとに結果が変わる（`Physics/PhysicsTest/FluidDeterministicTest.cpp` が回帰確認）。行を `std::vector<int>` ではなく 2 本のフラット配列で持つのは、毎ステップ作り直す前提で粒子ごとのヒープ確保が支配的コストになるため（`docs/issue/wcsph_parallel_scaling_profile.md` 4 節）。粒子側の受け口は `Space::NeighborIndexView`（`std::vector<int>` から暗黙変換できるので、テストが手書きの近傍リストを渡す形はそのまま使える）。
- **`viscosityCoe` は解像度に依存しない（＝粒子半径で振ってはいけない）**: `WCSPHParticle::solveViscosityForce()` は
  `viscosityCoe * Δv * ∇²W_visc * m_j` を力に足し `forwardTime()` が `力/ρ` にするので、教科書
  （Müller 2003 の `μ Σ m_j Δv/ρ_j ∇²W`）の `1/ρ_j` が抜けているぶん `viscosityCoe ≡ μ/ρ` ——
  **動粘性係数（m²/s）そのもの**である。SPH のラプラシアン近似を通すと加速度は `viscosityCoe · ∇²v` で
  `h` に依存しない（実測: 物理的に同一のシーンを解像度 2 倍で回しても、同じ値なら減衰は 5% 以内で一致）。
  `particle_radius`/`time_step` のように「下見ティアは本番の N 倍」と振ると、下見の実効粘性が本番の
  N 倍になり別の絵になる（`docs/issue/water_sphere_showcase_emitter_instability.md` 11.2 節）。
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
- **新しい流体ソルバーを追加する場合**: `ISPHSolver` を実装し、Two-Way 結合をサポートしないなら `addRigidBoundaryParticles`/`addSoftBoundaryParticles`/`supportsTwoWayCoupling()` はデフォルト実装（no-op）のままでよい。サポートする場合は `IBoundaryParticles*` のリストを rigid 用・soft 用の2本持ち（`clearRigidBoundaryParticles()`/`clearSoftBoundaryParticles()` を独立に保つため）、両リストに対して同一の `addBoundaryParticleDensity()`/`addBoundaryParticlePressure()` を呼ぶ形にする（DFSPHSolver/PBSPHSolver/WCSPHSolver 参照）。 `ISPHSolver::setBoundarySpheres()`/`setBoundaryPlates()` も同様にデフォルトが no-op で、実装しているのは WCSPHSolver のみ（`SphereBoundary`/`PlateBoundary` の密度寄与を安全に足すには DFSPH の α／PBSPH の制約勾配に相当する counterpart が要るため、上記「DFSPH の境界結合は…」と同じ理由で DFSPH/PBSPH は非対応）。実装する場合は `boundarySpheres_`/`boundaryPlates_` を値で保持し、`addBoundaryForce()`/`addBoundaryDensity()` の平面ループの隣に並べる（WCSPHSolver 参照）。
- **PhysicsView と Universe の使い分けに注意**: 新しいアプリ/機能で Rigid/Fluid/SoftBody 結合が必要な場合は `Physics::PhysicsSolver` を使うのが標準。`PhysicsView`の`FluidWorld`/`RigidBodyWorld`/`SoftBodyWorld`も内部では単一の`PhysicsSolver`を共有する（上記参照）が、Universeと異なり`PhysicsSolver::step()`/`setRunning()`は使わず独自に個別Play/Pauseをオーケストレーションしている点に注意（fluid/rigid/softを独立制御する必要がないアプリではUniverseパターン＝`PhysicsSolver::step()`/`stepUnconditional()`/`setRunning()`をそのまま使う方が単純）。
- **SPH の長さ単位はシーンごとの暗黙の取り決め（`docs/todo/PLAN_sph_scale_invariance.md` 参照）**: `Physics/Physics` 内部のどの型も長さの単位を明示的には定義していない。本コードベースの慣例的な既定シーン（`radius=1` 等）は暗黙に「1 unit ≒ 1m」を仮定しており、重力の既定値（`DFSPHSolver`/`PBSPHSolver`/`WCSPHSolver` の `externalForce{0,-9.8f,0}`・`.fsscene` の `gravityY="-9.8"` 等）はこの前提のもとでの現実の重力加速度（9.8 m/s²）をそのまま採用している。**シーン全体の長さスケールを変える場合**（例: 粒子半径を 1.0 → 0.01 にして「1cm 粒子」を表現する）は、以下も比例して再スケールしないと同じ相対挙動にならない: 重力・`RigidBoundary::penaltyStiffness_`（[Phase 2](docs/todo/PLAN_sph_scale_invariance.md) 以降は `estimateStiffness(dt)` で `dt` から自動導出可能）・`WCSPHFluid::pressureCoe`/`PBSPHFluid::stiffness`（[Phase 1](docs/todo/PLAN_sph_scale_invariance.md) 以降は `estimatePressureCoe()`/`setPressureCoeFromScale()` で自動導出可能）・`maxTimeStep`/`boundaryTimeStep`（[Phase 6](docs/todo/PLAN_sph_scale_invariance.md) 時点では手動比例のまま、既定値 `0.01f` は `radius=1` の CFL 見積もり相当）。DFSPH/PBSPH/WCSPH いずれも `setEffectLength()` を呼ばずに `simulate()` すると effectLength が `0.f` のまま（[Phase 5](docs/todo/PLAN_sph_scale_invariance.md)）で no-op になる点にも注意。
