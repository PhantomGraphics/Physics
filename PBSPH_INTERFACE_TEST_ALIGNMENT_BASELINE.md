# PBSPH インタフェース整合 — フェーズ1 ベースライン記録

`PBSPH_INTERFACE_TEST_ALIGNMENT_PLAN.md` フェーズ1「ベースラインと契約表の固定」の記録。
以降の変更はこの記録との差分でレビューする。

## 1. `ISPHSolver` override 契約表（2026-09-02 時点）

| メンバ | ISPHSolver 既定 | WCSPHSolver | DFSPHSolver | PBSPHSolver |
|---|---|---|---|---|
| `simulate(dt,maxIter)` | pure | override | override | override |
| `setExternalForce` | pure | override | override | override |
| `setTimeStep` | no-op | override | override | override |
| `setMaxSubstep` | `setTimeStep` 委譲 | **override**(alias) | **override**(alias) | 既定のまま（未 override） |
| `setEffectLength` | no-op | **override**（登録 fluid へ伝播） | **override**（登録 fluid へ伝播） | **未実装** |
| `setBoundary(box)` | no-op | override | override | override |
| `setBoundaryPlanes` | no-op | override | override | override |
| `setBoundarySpheres` | no-op | override | override | **未実装**（既定 no-op） |
| `setBoundaryPlates` | no-op | override | override | **未実装**（既定 no-op） |
| `setShapeBoundaries` | no-op | override | override | **未実装**（既定 no-op） |
| `addShapeBoundary` | no-op | override | override | **未実装**（既定 no-op） |
| `clearShapeBoundaries` | no-op | override | override | **未実装**（既定 no-op） |
| `getLastSolveStats` | `{}` | **override**（`lastSolveStats_`） | **override**（`lastSolveStats_`） | **未実装**（既定 `{}`） |
| `setBoundaryDampingRatio` | no-op | override | override | 既定 no-op を意図的に維持（計画 17 節） |
| `getBoundaryDampingRatio` | `0.f` | override | override | 既定 `0.f` を維持 |
| `clear` | pure | override | override | override |
| `getKernel` | nullptr | override | override | override |
| `getRestDensity` | `0.f` | override | override | override |
| `getParticleCount` | pure | override | override | override |
| `getParticlePositions/Velocities/Densities` | pure | override | override | override |
| `addRigidBoundary` / `clearRigidBoundaries` | pure | override | override | override |
| `addRigidBoundaryParticles` / `clearRigidBoundaryParticles` | no-op | override | override | override |
| `supportsTwoWayCoupling` | `false` | `true` | `true` | `true` |
| `addSoftBoundaryParticles` / `clearSoftBoundaryParticles` | no-op | override | override | override |

### PBSPH に不足している override（計画で追加予定）

- フェーズ2: `setMaxSubstep`（明示 alias）、`setEffectLength`（登録 fluid へ伝播）
- フェーズ3: `getLastSolveStats` + `lastSolveStats_`、設定検証（`dt<=0`/`maxIter`/kernel 未設定/複数 fluid の
  effectLength・rest density 不一致）
- フェーズ4: `setBoundarySpheres` / `setBoundaryPlates` / `setShapeBoundaries` / `addShapeBoundary` /
  `clearShapeBoundaries`（+ 内部 shape ストレージの `std::shared_ptr<IShapeBoundary>` 化）

### 意図的に据え置く項目

- `setBoundaryDampingRatio` / `getBoundaryDampingRatio`: PBSPH は予測位置を hard clamp するため
  ペナルティばねのリバウンドエネルギーを保持しない。物理的意味が定まるまで no-op 維持（計画 17・119 節）。

## 2. ベースラインテスト結果（変更前）

### PBSPH 関連ユニットテスト

`PhysicsTest.exe --gtest_filter='PBSPH*:*PBSPH*'` → **32 tests, 全 PASS**（283 ms）

内訳:
- `PBSPHFluidTest` 15 本（getter/setter, particle add, bounding box, emitter 6 本,
  outflow 3 本, effect length, scale invariance 2 本, finite state）
- `PBSPHSolverTest` 3 本（`GravityDropsYAfterOneStep`, `ClearAndReAddSimulatesCorrectly`,
  `TwoFluidsAllParticlesStayFinite`）
- `PBSPHRigidBoundaryParticleTest` / `PBSPHSoftBoundaryParticleTest`（別ファイル、合計に含む）
- `PhysicsSolverTest.Build_PBSPH_CreatesParticles` 1 本
- `TwoWayCouplingTest.PBSPH_*` 3 本

### PBSPH シナリオ（PhysicsView）

`run_physics_scenarios.ps1 -Filter '1*_fluid_pbsph_*'` → **3/3 PASSED**（total 59.0s）

| シナリオ | 結果 | 時間 |
|---|---|---|
| `11_fluid_pbsph_pool_settle` | PASSED | 11.3s |
| `15_fluid_pbsph_dam_break` | PASSED | 17.3s |
| `17_fluid_pbsph_small_scale_regression` `[slow]` | PASSED | 30.4s |

### 全 `PhysicsTest`

（フェーズ1実行時点の記録。full suite の PASS/FAIL 件数はここに追記。）
- 実行コマンド: `.\build\windows-debug\Physics\PhysicsTest.exe`
- 結果: **344 tests / 58 test cases, 全 PASS**（331,965 ms）

## 3. 完了条件チェック

- [x] 変更前に全 PBSPH テストが通る（32/32）
- [x] PBSPH シナリオが通る（3/3）
- [x] override 契約表を作成（上記 1 節）
- [x] full suite のベースラインを記録（344/344 PASS）

## 4. 共通契約テスト・マトリクス（フェーズ6完了時点）

`✓` = solver ごとの同名（または直接対応する）テストが存在。`—` = 不在（意図的分離、
または未整備）。

### `*SolverTest` / `*FluidTest`（共通契約）

| 契約項目 | WCSPH | DFSPH | PBSPH |
|---|---|---|---|
| Fluid getter round-trip | `GettersReturnSetValues` | `PublicFieldsSetAndRead` | `GettersReturnSetValues` |
| Create particle → count 増 | `CreateParticleIncreasesCount` | `AddParticleIncreasesCount` | `AddParticleIncreasesCount` |
| Bounding box | `BoundingBoxContainsAllParticles` | ✓ | ✓ |
| Emitter: no-op / rate / maxParticles / velocity dir / clear | ✓×5 | ✓×5 | ✓×5 |
| Outflow: no-op / inside-only / clear | ✓×3 | ✓×3 | ✓×3 |
| `setEffectLength` が kernel 更新 | (EstimatePressureCoe 系) | `EffectLengthUpdatesKernel` | `EffectLengthUpdatesKernel` |
| `ISPHSolver::setEffectLength` が登録 fluid 伝播 | `*` | `CommonEffectLengthInterfaceUpdatesRegisteredFluid` | `CommonEffectLengthInterfaceUpdatesRegisteredFluid` |
| `simulate` は frame 時間を進める（maxSubstep で gate されない） | `*` | `SimulateAdvancesRequestedFrameDurationNotMaxSubstep` | `SimulateAdvancesRequestedFrameDurationNotMaxSubstep` |
| 共通 solve stats 報告 | `ReportsFrameAdvanceThroughCommonSolveStats` | (同 Simulate… 内) | `ReportsFrameAdvanceThroughCommonSolveStats` |
| kernel 不一致 fluid を状態変更なしで拒否 | `RejectsFluidsWithMismatchedKernelConfiguration` | `RejectsFluidsWithMismatchedKernelConfiguration` | `RejectsFluidsWithMismatchedKernelConfiguration` |
| rest density 不一致を拒否 | — | — | `RejectsFluidsWithMismatchedRestDensity` |
| dt/maxIter 不正を状態変更なしで拒否 | (同上) | (同上) | `RejectsInvalidTimeStepWithoutTouchingParticleState` |
| effectLength 未設定なら no-op | `Simulate_IsNoOpWhenEffectLengthNeverSet` | — | `SimulateIsNoOpWhenEffectLengthNeverSet` |
| 静的 fluid は重力で動かない | `StaticFluidDoesNotMoveUnderGravity` | — | `StaticFluidDoesNotMoveUnderGravity` |
| 動的+静的の共存 | `DynamicFluidFallsWhileStaticFluidStaysFixed` | — | `DynamicFluidFallsWhileStaticFluidStaysFixed` |
| 複数 fluid で全粒子有限 | `TwoFluidsAllParticlesStayFinite` | — | `TwoFluidsAllParticlesStayFinite` |
| clear が登録 fluid を全消去 | (`GetFluidsReturnsRegisteredFluids`) | — | `ClearRemovesAllRegisteredFluids` |
| 共通 particle getter の順序・個数 | — | — | `ParticleGettersAreConsistentInOrderAndCountAcrossFluids` |
| Sphere 境界＝共通 shape 経路 | `ParticlesSettlingInsideSphereContainerStayWithinRadius` | `SphereBoundaryUsesCommonShapeInterface` | `SphereBoundaryUsesCommonShapeInterface` |
| Plate 境界＝共通登録・補正経路 | `ParticlesRestingOnPlateReachRestDensity` / `WaterRunsOffThePlateEdge` | `PlateBoundaryUsesSameRegistrationAndForcePath` | `PlateBoundaryUsesSameRegistrationAndCorrectionPath` / `PlateBoundaryLetsParticlePastItsEdgeFallFree` |
| shape の add/clear＝共通 interface | — | `ShapeBoundariesCanBeAddedAndClearedThroughCommonInterface` | `ShapeBoundariesCanBeAddedAndClearedThroughCommonInterface` |
| typed/generic 登録で同一補正 | — | — | `PlaneRegisteredAsGenericShapeMatchesTypedPlane` |
| 遠方 shape は no-op | (`NoRegisteredSpheres…`) | — | `DistantSphereBoundaryIsNoOp` |

`*` WCSPH は `setEffectLength`/frame-advance を個別テスト名では持たないが `WallContributesDensity…` 等の
セットアップで暗黙にカバー。

### Two-Way 境界テスト（`*RigidBoundaryParticleTest` / `*SoftBoundaryParticleTest`）

| 契約項目 | WCSPH | DFSPH | PBSPH |
|---|---|---|---|
| 侵入粒子を押し戻し＋反力蓄積 | ✓ | ✓ | ✓（fluid 側は position correction） |
| 遠方粒子は無補正 | ✓ | ✓ | ✓ |
| 境界リスト空なら no-op | `No{Rigid,Soft}Boundaries_IsNoOp` | ✓ | ✓ |
| `supportsTwoWayCoupling()==true` | ✓（rigid） | — | ✓（rigid） |
| clear は該当種類のみ解除 | — | — | `Clear{Rigid,Soft}BoundaryParticles_Leaves…Coupled` |
| 反復数に反力が比例して過大化しない | — | (frameShare で担保) | `BoundaryReactionDoesNotInflateWithIterationCount` |

### solver 固有テスト（意図的に分離・共通化しない）

- **PBSPH**: `PositionCorrectionOverRadiusIsScaleInvariant`,
  `LambdaDoesNotSaturateAcrossEffectLengthAndRestDensityScales`, `NewParticlesHaveFiniteState`,
  `GravityDropsYAfterOneStep`, `ClearAndReAddSimulatesCorrectly`
- **DFSPH**: `CalculateRestDensity*`（4本）, `IsDivergenceErrorAcceptable_*`（2本）,
  `WallPenetrationDoesNotGrowWithImpactSpeed`, `EmittedParticlesAtSceneRadiusStayFinite…`
- **WCSPH**: `EstimatePressureCoe*`, `SetPressureCoeFromScale…`, `ViscousDampingIsResolutionIndependent…`,
  `SurfaceTension*`（3本）, `InteriorParticleNormal…`, `Wall*Density*`（6本）,
  `CoincidentParticlesDoNotContaminate…`, `PlateDensityTapersAtTheRim`, `TwoOverlappingPlates…`

## 5. 進捗（フェーズ別）

| フェーズ | 内容 | 状態 | PhysicsTest 合計 |
|---|---|---|---|
| 1 | ベースライン・契約表 | 完了 | 344 |
| 2 | `setMaxSubstep` / `setEffectLength` | 完了 | 347 |
| 3 | solve stats / 設定検証 + PBSPHFluid 既定 kernel=0 | 完了 | 352 |
| 4 (u3) | shape ストレージ・plane 移行 | 完了 | 352 |
| 4 (u4) | Sphere 対応 | 完了 | 359 |
| 4 (u5) | Plate / generic shape 対応 | 完了 | 363 |
| 5 | Two-Way テスト対称化 | 完了 | 369 |
| 6 | 基本契約テストの共通マトリクス化 | 完了 | 373 |
| 7 | 統合確認・文書更新 | 完了 | 373 |

## 6. フェーズ7 統合確認結果

### ビルド
- `PhysicsCore` / `PhysicsTest` / `PhysicsView` / `FlameView` / `FluidGPUVkCore` / `FluidRendererCore`
  全て再ビルド成功（VS2026 Dev Shell）。

### テスト
- `*PBSPH*` フィルタ: **58/58 PASS**（ベースライン 32 → +26）
- 全 `PhysicsTest`: **373/373 PASS**（ベースライン 344 → +29、回帰ゼロ）

### シナリオ
- PBSPH を使う全シナリオ（`11_fluid_pbsph_pool_settle` / `15_fluid_pbsph_dam_break` /
  `17_fluid_pbsph_small_scale_regression` / `60_bound_mesh_floor`）を**フェーズ7 の変更込みで再ビルドした
  PhysicsView.exe** に対し直接実行 → **4/4 PASSED**。
- `run_physics_scenarios.ps1` の一括実行は `00_smoke_startup` の `SetRunning:true`（auto-advance ループ）
  で**この作業と無関係にハングする既知の環境問題**がある:
  - 本 PBSPH 変更を `git stash` で全て退避したクリーンビルドでも同一箇所でハングする（＝本変更起因ではない）。
  - 旧 `C:\Dev\Crystal2024\build`（Sep 1、submodule の直近コミット群より前）の PhysicsView.exe では
    `00_smoke_startup` は通る。現行 HEAD（`24d2f1b`）を再ビルドすると通らない。
  - `00_smoke_startup` は DFSPH 既定＋`SetRunning` ループのみで PBSPH コードを一切通らない。
  - ランナーの `repoRoot` 検出（`Phantom2026.sln` を上方探索）が `C:\Dev\Crystal2024` に解決し、
    stale な exe と存在しない `${repo_root}/Physics/...` パスを使う二重チェックアウト問題も別途ある
    （`60_bound_mesh_floor` の STL 読み込み失敗の原因。パス補正すれば通る）。
  - → PBSPH 経路の回帰確認は上記 4 本の直接実行 + 全 `PhysicsTest` 373/373 で担保。一括ランナーの
    ハングは本整合作業のスコープ外（別途トリアージ要）。

### `ISPHSolver` 経由の設定箇所レビュー（PhysicsView `FluidWorld`）
- `reregisterBoundarySpheres()` は既に `fluidSolver_->setBoundarySpheres()`（インタフェース）
  経由で呼んでおり、PBSPH 専用 switch は無い。今回 PBSPH の実装が no-op → 有効になったことで、
  `AddBoundarySphere` コマンド／`ControlPanel` の Sphere 境界が PBSPH でもそのまま機能するようになった
  （コード変更不要）。
- `createPBSPH()` / `createDFSPH()` / `createWCSPH()` は元々同型（`setTimeStep`/`setBoundary`/
  `setExternalForce`/`add`）。削減対象の PBSPH 専用分岐は見当たらなかった。

### 文書・コメント更新
- `Physics/ISPHSolver.h`: クラス doc の実装ソルバー列挙に WCSPH を追加、Two-Way / analytic shape が
  WCSPH/DFSPH/PBSPH 実装済みである旨に修正。`setBoundarySpheres`/`setBoundaryPlates` の「PBSPH 未対応」
  記述を削除（フェーズ4で対応済み）。
- `Physics/PBSPHSolver.h`: `setBoundary()` doc に `addShapeBoundaryConstraint()`（境界密度＋制約勾配の
  対）を機構リストへ追加。
- `Physics/CLAUDE.md`: 解析境界の節と「新しい流体ソルバーを追加する場合」の節を
  「Plane/Sphere/Plate/generic を WCSPH/DFSPH/PBSPH が実装」に更新。
- `Physics/PhysicsView/FluidWorld.h` / `CommandDispatcher.h`: Sphere 境界・Track A/B のサポート
  ソルバー列挙を実態（WCSPH/DFSPH/PBSPH）に修正。

## 7. 最終受け入れ基準チェック

- [x] PBSPH が `ISPHSolver*` 経由で effect length・最大 substep・全 analytic shape・diagnostics を扱える
- [x] 無効設定を状態変更なしで拒否し、`validConfiguration=false` で通知する
- [x] Plane/Sphere/Plate/generic shape の登録・追加・全消去が共通 API で機能する
- [x] rigid/soft Two-Way の登録・未登録・clear・反力の契約がテストされる
- [x] 共通契約テストが WCSPH/DFSPH と対応（§4 マトリクス）、PBSPH 固有数値テストは維持
- [x] 全 `PhysicsTest` と対象 PBSPH scenario に新規回帰なし
