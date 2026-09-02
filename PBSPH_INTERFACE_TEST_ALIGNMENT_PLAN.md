# PBSPH インタフェース・テスト整合計画

## 目的

`PBSPHSolver` / `PBSPHFluid` を、現在の `WCSPH`・`DFSPH` が実装している `ISPHSolver` の契約とテスト構成に揃える。PBSPH 固有の位置拘束法は維持し、アルゴリズムを WCSPH/DFSPH と同一化することは目的にしない。

## 現状と差分

現時点で PBSPH は、共通の `simulate()`、外力、時間刻み、平面境界、One-Way/Two-Way 境界、粒子状態 getter を実装している。一方、WCSPH/DFSPH と比較すると次が不足している。

- `setMaxSubstep()` と `setEffectLength()` の明示的な override
- `SPHSolveStats` の保存・返却、および入力/複数 fluid 設定の検証
- `IShapeBoundary` を使う Sphere/Plate/任意形状境界の共通登録・消去経路
- solver の共通インタフェース契約を直接確認する PBSPH テスト
- Two-Way rigid/soft 境界の「未登録なら no-op」と capability の対称なテスト

なお `setBoundaryDampingRatio()` は、ペナルティばねを積分する WCSPH/DFSPH と異なり、PBSPH は予測位置を拘束・クランプするため、現状の no-op が意図された契約である。単なる API 対称性のために状態を追加しない。

## 方針

1. `ISPHSolver` の意味論を基準にし、共通呼び出し側から PBSPH だけを特別扱いしなくてよい状態にする。
2. 既存の PBSPH 数値挙動を先に characterization test で固定し、小さな単位でインタフェースを追加する。
3. Shape boundary は、密度制約の分子だけでなく lambda の分母となる拘束勾配にも同じサンプルを反映する。WCSPH/DFSPH の登録・走査構造は再利用するが、力の式は PBSPH の位置補正として実装する。
4. 複数 fluid は、WCSPH/DFSPH と同様に kernel effect length と rest density の互換性を検証する。非互換時は粒子状態を変更せず `validConfiguration=false` を返す。
5. 既存の public helper とテスト名は、外部利用を壊す理由がない限り維持する。

## 実施フェーズ

### 1. ベースラインと契約表の固定

- 対象ファイル: `Physics/ISPHSolver.h`, `Physics/PBSPHSolver.*`, `Physics/PBSPHFluid.*`, `PhysicsTest/PBSPHSolverTest.cpp`, PBSPH の rigid/soft boundary test。
- PBSPH/WCSPH/DFSPH の override 一覧をテストレビュー用のチェックリストにする。
- 現行 PBSPH テストと PBSPH 関連シナリオを実行し、既知の許容値、反復回数、粒子配置をベースラインとして記録する。
- 完了条件: 以降の変更前に全 PBSPH テストが通り、失敗中のシナリオがあれば今回の回帰と区別できる。

### 2. 時間刻み・kernel 設定 API の整合

- `PBSPHSolver::setMaxSubstep(dt)` を `setTimeStep(dt)` の明示的 alias として override する。
- `PBSPHSolver::setEffectLength(length)` を追加し、登録済み全 `PBSPHFluid` に伝播する。
- `simulate(dt, maxIter)` が要求された frame 時間を進め、`maxTimeStep` は adaptive substep の上限としてだけ使われることをコメントとテストで固定する。
- 追加テスト:
  - `CommonEffectLengthInterfaceUpdatesRegisteredFluid`
  - `SimulateAdvancesRequestedFrameDurationNotMaxSubstep`
  - 未登録 fluid に対する両 setter の安全な no-op
- 完了条件: `ISPHSolver*` 経由で設定しても PBSPH 具象 API と同じ結果になる。

### 3. 共通 solve diagnostics と設定検証

- `PBSPHSolver` に `lastSolveStats_` と `getLastSolveStats()` を追加する。
- 各 `simulate()` 冒頭で stats を初期化し、正常時に `substeps`, `advancedTime`, `converged`, `validConfiguration` を更新する。
- PBSPH の constraint projection iteration を `densityIterations` に計上し、`divergenceIterations` は 0 のままとする。この意味を `SPHSolveStats` のコメントにも明記する。
- `dt <= 0`、`maxIter` の不正値、kernel 未設定/非有限値、複数 fluid 間の effect length/rest density 不一致を検証する。拒否時は状態を部分更新しない。
- 追加テスト:
  - `ReportsFrameAdvanceThroughCommonSolveStats`
  - `RejectsFluidsWithMismatchedKernelConfiguration`
  - `SimulateIsNoOpWhenEffectLengthNeverSet`
  - 不正入力時に粒子位置・速度が不変で stats が invalid になること
- 完了条件: WCSPH/DFSPH と同じ診断フィールドを共通 UI/runner から解釈できる。

### 4. analytic shape boundary の共通化

- PBSPH の `std::vector<PlaneBoundary>` を、WCSPH/DFSPH と同じ所有権モデルの `std::shared_ptr<IShapeBoundary>` 群へ段階的に移行する。
- `setBoundaryPlanes`, `setBoundarySpheres`, `setBoundaryPlates`, `setShapeBoundaries`, `addShapeBoundary`, `clearShapeBoundaries` を実装する。
- `IShapeBoundary::sample()` の同一結果から、次の二項を必ず対で計算する。
  - 境界による密度制約への寄与
  - `calculateLambda()` の分母に必要な拘束勾配
- 得られた lambda に基づく PBSPH の位置補正を適用し、その後の hard clamp は tunneling 防止用の安全網として維持する。Sphere/Plate を box の単純 clamp に押し込めない。
- shape の active/inactive、符号規約、rim taper、重複 shape の寄与上限は WCSPH/DFSPH のテスト期待と揃える。
- 追加テスト:
  - `SphereBoundaryUsesCommonShapeInterface`
  - `PlateBoundaryUsesSameRegistrationAndCorrectionPath`
  - `ShapeBoundariesCanBeAddedAndClearedThroughCommonInterface`
  - sphere 内への保持、plate 上の保持と端からの流出、遠方 shape の no-op
  - plane と generic `IShapeBoundary` 登録で同等の補正になること
- 完了条件: PBSPH でも typed/generic API のどちらから登録しても同じ shape 評価経路を通り、既存 plane 境界テストが退行しない。

### 5. Two-Way boundary test の対称化

- `PBSPHRigidBoundaryParticleTest.cpp` と `PBSPHSoftBoundaryParticleTest.cpp` の構成・fixture・許容値表現を WCSPH/DFSPH counterparts と揃える。
- 既存の「侵入時に流体を押し戻し反力を蓄積」「遠方では補正なし」に加え、次を追加する。
  - `NoRigidBoundaries_IsNoOp`
  - `NoSoftBoundaries_IsNoOp`
  - `SupportsTwoWayCoupling_IsTrue`
  - clear 後は該当する種類だけが解除され、rigid/soft の他方は維持されること
  - 複数 substep でも反力が substep 数に比例して過大化しないこと
- PBSPH 固有の比較量は force ではなく position correction とし、境界側反力だけ force として検証する。
- 完了条件: 3 solver の境界テストが同じ契約項目を網羅しつつ、各 solver の物理量の違いを明示している。

### 6. fluid/solver 基本テストの共通マトリクス化

- PBSPH に不足する solver 契約テストを追加する: 登録 fluid の getter/clear、静的 fluid、動的+静的 fluid、複数 fluid の有限性、共通 particle getter の順序と個数。
- emitter/outflow の共通ケースは 3 solver で同じ命名と入力値に揃える。solver 固有テスト（PBSPH の lambda/scale invariance、DFSPH の divergence、WCSPH の EOS/表面張力）は分離したままにする。
- 重複コードが大きくなった場合のみ、`PhysicsTest` 内に typed test または小さな factory/helper を導入する。本体コードにテスト専用分岐は入れない。
- 完了条件: 共通契約の追加漏れを solver ごとの同名テスト一覧で確認できる。

### 7. 統合確認と文書更新

- PBSPH filter、全 `PhysicsTest`、PBSPH を使う短時間 scenario、最後に通常の scenario suite（`known-fail`/`slow` の扱いは既存運用に従う）の順で実行する。
- `FluidWorld`/factory/PhysicsView が `ISPHSolver` 経由で設定する箇所を確認し、PBSPH 専用 switch が今回追加した共通 API で削減可能かを評価する。削除は挙動が同一と確認できる範囲だけにする。
- `ISPHSolver.h` と `PBSPHSolver.h` のコメントから「PBSPH は未対応」とする古い記述を更新する。
- 完了条件: 新旧 PBSPH テスト、全 solver 共通契約テスト、対象 scenario が通り、API コメントと実装の対応が一致する。

## 推奨する変更単位

1. `setMaxSubstep` / `setEffectLength` とテスト
2. solve stats / configuration validation とテスト
3. shape storage と plane の移行（既存挙動維持）
4. sphere 対応とテスト
5. plate/generic shape 対応とテスト
6. Two-Way・基本契約テストの対称化
7. 統合確認と文書整理

各単位を独立してレビュー・revert 可能にし、特に shape 対応と diagnostics を同じ変更に混在させない。

## 非目標・注意点

- PBSPH を圧力ばね方式または DFSPH の divergence solve に変更しない。
- 数値結果を solver 間で完全一致させない。比較対象は API 契約、不変条件、符号、有限性、スケール則とする。
- `setBoundaryDampingRatio()` は PBSPH の位置拘束に対応する明確な物理的意味が決まるまで no-op を維持する。
- Shape boundary 実装では密度寄与だけを先行投入しない。拘束勾配を欠くと lambda が不安定になるため、必ず一つの変更単位で追加する。
- 公開 API の typo（例: 既存の `setVicsosity`, private helper の `addBoundadryPressure`）の改名は互換性問題を伴うため、この整合化とは別課題にする。

## 最終受け入れ基準

- PBSPH が `ISPHSolver*` 経由で effect length、最大 substep、全 analytic shape、diagnostics を扱える。
- 無効設定を状態変更なしで拒否し、その理由を少なくとも `validConfiguration` で通知できる。
- Plane/Sphere/Plate/generic shape の登録、追加、全消去が共通 API で機能する。
- rigid/soft Two-Way の登録・未登録・clear・反力の契約がテストされる。
- 共通契約テストは WCSPH/DFSPH と対応関係が分かり、PBSPH 固有数値テストは維持される。
- 全 `PhysicsTest` と対象 PBSPH scenario に新規回帰がない。
