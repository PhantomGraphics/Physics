# PBSPH インタフェース・テスト整合 実装レビュー

## レビュー対象

- 実装コミット: `e30d48e` (`Align PBSPHSolver with the common ISPHSolver contract`)
- 基準文書: `PBSPH_INTERFACE_TEST_ALIGNMENT_PLAN.md`
- 主な対象:
  - `Physics/ISPHSolver.h`
  - `Physics/PBSPHSolver.h`
  - `Physics/PBSPHSolver.cpp`
  - `Physics/PBSPHFluid.cpp`
  - `PhysicsTest/PBSPHSolverTest.cpp`
  - `PhysicsTest/PBSPHRigidBoundaryParticleTest.cpp`
  - `PhysicsTest/PBSPHSoftBoundaryParticleTest.cpp`

## 結論

ビルドおよび既存テストは成功しており、共通 API、solve diagnostics、analytic shape boundary、Two-Way coupling のテストは大幅に拡充されている。

ただし、以下の理由から計画どおりの実装が完全に完了したとは判断しない。

1. 複数 fluid の非有限設定を拒否できず、`NaN` がシミュレーションへ流入する可能性がある。
2. `setMaxSubstep()` の意味が計画上の adaptive substep 上限と一致していない。
3. shape boundary の重要な「密度寄与と拘束勾配の対」をテストが直接保証していない。

判定: **要修正** → 対応済み（下記「対応状況（2026-09-03）」節を参照）

## 指摘事項

### 1. 高: 2番目以降の fluid の非有限設定を拒否できない

対象: `Physics/PBSPHSolver.cpp` の `PBSPHSolver::simulate()` 設定検証部分。

複数 fluid の互換性検証は、次のような差分比較で実装されている。

```cpp
std::abs(value - commonValue) > tolerance
```

2番目以降の fluid の effect length または rest density が `NaN` の場合、比較結果は false になり、不正設定が検出されない。その後の密度、拘束勾配、lambda、位置計算へ `NaN` が伝播する可能性がある。

また、先頭 fluid は `fluids.front()->getKernel()` の呼び出し前に null 検証されていないため、`add(nullptr)` が呼ばれた場合はクラッシュする。

#### 必要な修正

各登録 fluid をデリファレンスする前に、次を明示的に検証する。

- fluid ポインタが null でない
- kernel ポインタが null でない
- effect length が有限かつ正である
- rest density が有限かつ正である
- 共通値との差が許容範囲内である

検証失敗時は、粒子状態を変更せずに `lastSolveStats_.validConfiguration = false` として終了する。

#### 必要なテスト

- 先頭 fluid が null の場合に安全に拒否する
- 2番目の fluid の effect length が `NaN` / infinity の場合に拒否する
- 2番目の fluid の rest density が `NaN` / infinity / 0以下の場合に拒否する
- 拒否時に全 fluid の位置・速度が不変である

### 2. 中: `setMaxSubstep()` が計画上の契約と一致しない

対象:

- `Physics/PBSPHSolver.h` の `setTimeStep()` / `setMaxSubstep()`
- `Physics/PBSPHSolver.cpp` の `simulate()` / `calculateTimeStep()`
- `PhysicsTest/PBSPHSolverTest.cpp` の `SimulateAdvancesRequestedFrameDurationNotMaxSubstep`

実装では PBSPH が `simulate(dt)` を常に単一ステップで積分し、`setMaxSubstep()` は `setTimeStep()` の単純な別名になっている。保存された `maxTimeStep` は主に境界ペナルティから位置補正へ変換するために使われ、実際のサブステップ上限としては使われていない。

さらに `calculateTimeStep()` は呼び出されておらず、内部で算出した CFL 値も使用せず、常に `maxTimeStep` を返している。

これは計画書にある以下の方針と一致しない。

- `maxTimeStep` は adaptive substep の上限とする
- `simulate(dt)` は要求された frame 時間を、必要なら複数 substep に分けて進める

現在のテストは「異なる `setMaxSubstep()` 値でも結果が同じ」ことを期待しており、計画との差異を仕様として固定している。

#### 必要な判断

次のどちらかに統一する。

1. 計画どおり PBSPH に substep 処理を実装し、`setMaxSubstep()` を上限として機能させる。
2. PBSPH は意図的に単一ステップと決定し、計画書および `ISPHSolver` の共通契約を修正する。この場合、境界補正用の時間刻みには `setMaxSubstep()` とは異なる名前・状態を用いることを検討する。

特に、analytic shape boundary の補正が `simulate(dt)` ではなく保存された `maxTimeStep` に依存する現在の API は、両者が異なる場合の意味を明確にする必要がある。

### 3. 中: shape boundary の数値契約をテストが保証していない

対象:

- `Physics/PBSPHSolver.cpp` の `addShapeBoundaryConstraint()`
- `PhysicsTest/PBSPHSolverTest.cpp` の Sphere/Plate/generic shape テスト

実装は `IShapeBoundary::sample()` の結果から、次を対で追加している。

- 境界による密度寄与
- lambda 分母に使われる拘束勾配

この設計自体は計画に沿っている。一方、追加されたテストは主に、最終位置が境界内に収まることや、typed/generic 登録の結果が一致することを確認している。

これらは penalty correction と `clampPosition()` だけでも成功するため、`addShapeBoundaryConstraint()` が削除されたり、密度または拘束勾配の一方だけになったりしても検出できない可能性が高い。

また、計画に記載された次のケースが不足している。

- plate の rim taper
- 重複 shape の寄与上限
- shape boundary による密度寄与
- 密度寄与と拘束勾配が同じ sample から対で適用されること

#### 必要なテスト

- 同じ侵入深度の plane と sphere で、境界密度・補正の符号とスケールが整合する
- plate 中央から rim に近づくほど寄与が滑らかに減少する
- 重複 shape で密度または補正が不当に発散しない
- 密度寄与があるケースで lambda と最終補正が有限である
- constraint-gradient 経路を欠く実装では失敗する回帰テストを設ける

テスト用 public API を増やさず観測できない場合は、friend fixture、限定的な test adapter、または複数粒子の結果から拘束項の効果を分離できるシナリオを使用する。

## 計画への適合状況

| 項目 | 判定 | 備考 |
|---|---|---|
| `setEffectLength()` の共通 API | 適合 | 登録済み fluid へ伝播する |
| `setMaxSubstep()` | 適合（単一ステップ仕様に統一） | alias。計画・`ISPHSolver` コメントを単一ステップ仕様へ修正済み |
| `SPHSolveStats` | 概ね適合 | single-step と反復数を報告する |
| 設定不整合の拒否 | 適合 | null・非有限・非正を登録順に関係なく状態変更なしで拒否（異常系テスト追加済み） |
| Plane/Sphere/Plate/generic shape 登録 | 適合 | 共通所有・走査経路が追加された |
| 密度寄与と拘束勾配の対 | 適合 | 専用の回帰テスト 4 本を追加（片方欠落・rim taper・重複 shape） |
| shape の追加・全消去 | 適合 | typed/generic リストを消去する |
| rigid/soft Two-Way no-op/clear | 適合 | 対称テストが追加された |
| Two-Way 反力の反復依存性 | 部分適合 | 有限性は確認するが、許容基準が緩く厳密な不変性ではない |
| static/dynamic/clear/getter 契約 | 適合 | PBSPH solver テストへ追加された |
| emitter/outflow 共通マトリクス | 部分適合 | 既存ケースは維持。WCSPHにあるdisk、重複防止、lattice cycleはPBSPHにない |
| 全 `PhysicsTest` | 適合 | 新規ビルドで373件成功 |
| PBSPH scenario | 未再検証 | コミット報告には成功記載あり。本レビューではunit testまで再実行 |

## 検証結果

### ビルド

既存の `out/build/x64-Debug/PhysicsTest.exe` はソース変更前の古いバイナリで、新規 PBSPH テストが含まれていなかった。そのため、`out/build/pbsph-review` に隔離した新規 Debug ビルドを作成して検証した。

- `PhysicsCore`: ビルド成功
- `PhysicsTest`: ビルド成功

### テスト

- PBSPH 関連: 47件中47件成功
- 全 `PhysicsTest`: 373件中373件成功
- 全テスト実行時間: 約119秒

既存テスト上の回帰は確認されなかった。

## 完了までの推奨順序

1. 全 fluid の null・有限値・正値検証を修正し、異常系テストを追加する。
2. PBSPH の `setMaxSubstep()` を実際のサブステップ上限にするか、単一ステップ仕様として計画と共通 API を更新する。
3. shape の密度・拘束勾配経路を単独で壊した場合に失敗するテストを追加する。
4. plate rim、重複 shape、境界密度のテストを追加する。
5. PBSPH 対象 scenario を新規ビルドで再実行する。
6. 上記完了後、全373件以上の `PhysicsTest` と対象 scenario を再実行する。

## 最終判定基準

次を満たした時点で計画完了と判断できる。

- 登録順に関係なく、null・非有限・非正の fluid 設定を状態変更なしで拒否できる。
- `setMaxSubstep()` の実装、テスト、計画書、`ISPHSolver` コメントの意味が一致する。
- shape boundary の密度寄与と拘束勾配の対が、専用テストで保証される。
- plate rim と重複 shape の重要な境界条件がテストされる。
- 全 unit test と対象 PBSPH scenario に回帰がない。

## 対応状況（2026-09-03）

判定「要修正」の 3 点をすべて対応した。

### 1. 高: 非有限・null fluid 設定の拒否 — 対応済み

`PBSPHSolver::simulate()` の設定検証を、差分比較の前に各 fluid を明示検証する形へ変更した
（`Physics/PBSPHSolver.cpp`）。`fluidScaleValid()` ローカルラムダが、登録順に関係なく次を拒否する。

- fluid ポインタが null（先頭 fluid をデリファレンスする前に検証。`add(nullptr)` でクラッシュしない）
- kernel ポインタが null
- effect length が非有限または 0 以下
- rest density が非有限または 0 以下

いずれも `lastSolveStats_.validConfiguration = false` として粒子状態を変更せずに終了する。

追加テスト（`PhysicsTest/PBSPHSolverTest.cpp`）:
`RejectsNullFrontFluidWithoutCrashing` / `RejectsNullSecondFluidWithoutTouchingFirst` /
`RejectsSecondFluidWithNonFiniteEffectLength`（NaN・infinity）/
`RejectsSecondFluidWithNonFiniteOrNonPositiveRestDensity`（NaN・infinity・0・負）/
`RejectsSingleFluidWithNonPositiveEffectLength`。全ケースで拒否後の位置・速度不変を確認。

### 2. 中: `setMaxSubstep()` の契約 — 単一ステップ仕様として統一（計画・コメント側を修正）

レビューの選択肢 2 を採った。PBSPH の position-based projection は 1 ステップ内の反復で
incompressibility を解く設計であり、DFSPH の adaptive substep 方式は非目標（「PBSPH を圧力ばね方式
または DFSPH の divergence solve に変更しない」と同じ理由）。実装はすでにこの単一ステップ方式で、
`setMaxSubstep()` は `setTimeStep()` の alias、`maxTimeStep` は境界ペナルティ加速度→位置補正の `dt^2`
係数としてのみ使われる（解析/剛体境界が未登録なら結果に影響しない）。

意味の不一致を解消するため、以下を単一ステップ仕様に合わせて更新した。

- `Physics/ISPHSolver.h` — `setTimeStep()`/`setMaxSubstep()` のコメントに、単一ステップ solver
  （WCSPH/PBSPH）は dt を substep 分割せず、この値がフレーム進行量を gate しないことを明記。
- `PBSPH_INTERFACE_TEST_ALIGNMENT_PLAN.md` Phase 2・最終受け入れ基準 — 「`maxTimeStep` は adaptive
  substep の上限」という記述を「PBSPH は意図的に単一ステップ」に修正。

`PBSPHSolver.h` のコメントと `SimulateAdvancesRequestedFrameDurationNotMaxSubstep`
（異なる `setMaxSubstep()` 値でも結果不変）はもともとこの仕様と一致しているため変更なし。

### 3. 中: shape boundary の数値契約テスト — 追加済み

`addShapeBoundaryConstraint()` が「密度寄与（PBF 制約の分子）」と「拘束勾配（`calculateLambda()` の
分母）」を同一 `sample()` から対で加えることを直接検証する回帰テストを追加した
（`PhysicsTest/PBSPHSolverTest.cpp`、いずれも近傍ゼロの孤立粒子で penalty/clamp から分離）。

- `ShapeBoundaryAddsToPenetratingParticleDensity` — 貫入した平面境界がある場合の最終密度が、境界
  なしより厳密に大きい（密度寄与の項を削ると失敗）。
- `ShapeBoundaryConstraintGradientKeepsLambdaCalibrated` — 唯一の拘束勾配寄与が境界由来のとき、
  lambda が有限かつ ±1 cap から十分内側（拘束勾配の項を削ると denom が 1e-6 床に落ちて lambda が
  cap に張り付き失敗）。
- `PlateBoundaryDensityContributionTapersTowardRim` — 同じ貫入深さで plate 中央 vs u 端の粒子を比較、
  rim taper（`getDensityWeight`）により端の密度寄与が中央より小さい。
- `OverlappingShapeBoundariesStayFiniteAndBounded` — 箱の角（3 平面が重なる領域）へ強く押し込んでも
  位置・速度が有限で角の近傍に留まる（重複寄与が発散しない）。

### 検証結果

- ビルド: `PhysicsCore` / `PhysicsTest` とも成功（`Phantom/build/windows-debug`、VS2026 v145）。
- 全 `PhysicsTest`: 382 件中 382 件成功（373 + 新規 9）。
- PBSPH 対象 scenario: `11_fluid_pbsph_pool_settle` / `15_fluid_pbsph_dam_break` /
  `17_fluid_pbsph_small_scale_regression` / `51_couple_rigid_fluid_twoway` / `12_fluid_wcsph_pool_settle`
  すべて成功。
- `60_bound_mesh_floor` は失敗するが、原因は `${repo_root}` が `Phantom/` を欠いた
  `C:/Dev/Crystal2024/Physics/...` に解決される環境側のパス問題（"failed to load mesh boundary"）で、
  本修正とは無関係（mesh boundary の読み込み経路には触れていない）。
