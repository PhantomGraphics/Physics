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

判定: **要修正**

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
| `setMaxSubstep()` | 不適合 | alias は追加されたが substep 上限として機能しない |
| `SPHSolveStats` | 概ね適合 | single-step と反復数を報告する |
| 設定不整合の拒否 | 要修正 | mismatch は拒否するが非有限値に穴がある |
| Plane/Sphere/Plate/generic shape 登録 | 適合 | 共通所有・走査経路が追加された |
| 密度寄与と拘束勾配の対 | 実装あり、検証不足 | 専用の回帰テストが必要 |
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
