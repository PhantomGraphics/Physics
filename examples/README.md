# Phantom::Physics — SPH サンプル集（GUI 不要）

`Physics/examples/` は、**ウィンドウも GPU も使わない**最小構成の SPH 流体シミュレーション
サンプルです。オープンソース版を触りはじめる際の「最初の一本」として、
`PhysicsCore`（静的ライブラリ）だけをリンクし、標準 C++ ライブラリと
`Phantom::Math` 以外に依存しません。

各サンプルは数百〜千粒子を数秒回し、粒子位置を連番 PLY
（`format binary_little_endian`、位置のみ）として書き出します。
Blender / MeshLab / CloudCompare / Houdini / Open3D などでそのまま開けます。

| サンプル | 内容 | 学べる API |
|---|---|---|
| `01_dam_break.cpp` | WCSPH のダムブレイク（水柱の崩壊）を PLY 連番で出力 | `WCSPHFluid` / `WCSPHSolver` の基本、境界箱、PLY 書き出し |
| `02_faucet.cpp` | 蛇口：エミッタで粒子を生成し、排水領域で削除。粒子数が増えて平衡する | `addEmitter()` / `updateEmitters()`、`addOutflowRegion()` / `removeOutflowParticles()` |
| `03_solver_interface.cpp` | 同じシーンを WCSPH と PBSPH で回す。ステップ処理は `ISPHSolver*` のみを触る | `ISPHSolver` 共通インターフェース、ソルバーの差し替え |

---

## ビルド

C++17 / CMake 3.20 以降 / Ninja。**Vulkan SDK は不要**（`PhysicsView` と違い、
このサンプル群は Vulkan/GLFW を一切参照しません）。

### 方法 A: Physics モジュール単体をビルドする（推奨）

```powershell
# 親リポジトリ (Phantom) のルートから
cmake -S Physics -B Physics/build_examples -DCMAKE_BUILD_TYPE=Release
cmake --build Physics/build_examples

# 実行ファイル: Physics/build_examples/examples/sph_example_0{1,2,3}_*.exe
```

`PhysicsCore` / `PhysicsTest` / `PhysicsView`（Vulkan が見つかれば）と一緒に
`examples/` もビルドされます。サンプルだけが要る場合は
`--target sph_example_01_dam_break` のように個別指定できます。
不要なら `-DPHYSICS_BUILD_EXAMPLES=OFF`。

### 方法 B: examples だけをスタンドアロンでビルドする

```powershell
cmake -S Physics/examples -B build/examples -DCMAKE_BUILD_TYPE=Release
cmake --build build/examples
```

この場合 `PhysicsCore` とその依存（`MathCore` / `SpaceCore` / `VolumeCore` /
`NumericsCore`）をソースから private にビルドします。親リポジトリ `Phantom` の
ディレクトリ構成（`cmake/PhantomCoreLibs.cmake` と `CGLib/` が同じ親にある）が前提です。

### 方法 C: リポジトリ全体のプリセット

```powershell
cmake --preset windows-debug
cmake --build --preset windows-debug --target sph_example_01_dam_break
```

> **注意（2026-09 時点）:** ルート `CMakeLists.txt` のクリーン構成が
> `CGLib` サブモジュールの二重 `add_subdirectory` で失敗する既知の状態です
> （`build/windows-debug` を作り直すと再現。`examples/` とは無関係）。
> 当面は方法 A か B を使ってください。

Debug ビルドは Release の 10 倍以上遅いので、サンプルを回すだけなら
`-DCMAKE_BUILD_TYPE=Release` を推奨します（`01` が Debug で ~18 秒 / Release で ~1.5 秒）。

---

## 実行

```powershell
# 01: ダムブレイク → ./out_dam_break/dam_0000.ply ...
.\Physics\build_examples\examples\sph_example_01_dam_break.exe  [出力ディレクトリ]

# 02: 蛇口 → ./out_faucet/faucet_0000.ply ...
.\Physics\build_examples\examples\sph_example_02_faucet.exe  [出力ディレクトリ]

# 03: ソルバー比較（PLY 出力なし、標準出力にサマリ）
.\Physics\build_examples\examples\sph_example_03_solver_interface.exe
```

`03` の出力例:

```
27-particle block settling in a box, 600 steps, one ISPHSolver* loop:
  WCSPH   n=27  minY 0.60 -> -3.00  restDensity 1.000  peakSpeed 8.46  finalSpeed 0.00  0.05s
  PBSPH   n=27  minY 0.60 -> -3.00  restDensity 1.000  peakSpeed 9.02  finalSpeed 0.00  0.10s
```

PLY 連番を Blender で開くには「File → Import → Stanford PLY」で
`*_0000.ply` を選び、"Import as sequence" 相当のアドオン、または
Geometry Nodes の "Import PLY" ノードを使います。

---

## コードの読みどころ

### 1. Fluid と Solver は別物、所有権は呼び出し側

```cpp
WCSPHFluid fluid;          // 粒子データ（SoA）と材料パラメータを所有
WCSPHSolver solver;
solver.add(&fluid);        // Solver は非所有ポインタを持つだけ
```

`*Fluid` が粒子の実体（位置・速度・密度）と材料係数を持ちます。
`*Solver` は 1 個以上の `*Fluid` を毎ステップ進めるだけで、**生ポインタしか持ちません**。
したがって Fluid は Solver より長生きさせる必要があります
（サンプルでは同じスコープに置くのが最も簡単）。この規約は
ライブラリ全体で共通です（`Physics/CLAUDE.md`「非所有ポインタの寿命」）。

### 2. 毎ステップの流れ

```cpp
fluid.updateEmitters(dt);      // (任意) 粒子を増やす
solver.simulate(dt, maxIter);  // 1 フレーム進める
fluid.removeOutflowParticles();// (任意) 粒子を減らす
```

`simulate(dt, maxIter)` の第 2 引数は制約反復回数です。WCSPH は単一パスなので
無視し、DFSPH / PBSPH が使います。エミッタも排水領域も登録しなければ
完全に no-op なので、`01` のように閉じた系ならこの 1 行だけで足ります。

### 3. `ISPHSolver` 共通インターフェース（`03`）

`WCSPHSolver` / `DFSPHSolver` / `PBSPHSolver` はいずれも `ISPHSolver` を実装します。
シーン構築時はソルバー固有 API を使いますが、構築後のステップ・問い合わせは

```cpp
void run(Phantom::Physics::ISPHSolver& s) {
    s.setEffectLength(...); s.setTimeStep(...);
    s.setExternalForce({0,-9.8f,0}); s.setBoundary(box, dt);
    for (int i = 0; i < N; ++i) s.simulate(dt, 4);
    auto positions = s.getParticlePositions();   // 具象型に触れない
}
```

だけで完結します。`PhysicsView` の CPU 経路や上位の統合アプリはこの形で
ソルバーを差し替えています。

### 4. 長さの単位とスケール

`Physics/Physics` の型は長さの単位を定義しません。サンプルは
**粒子半径 0.15 / カーネル支持長 1.0 / タイムステップ 0.005 s** という、
シナリオテスト（`PhysicsView/scenarios/11,12,15`）で検証済みの
「小スケール安定域」を使っています。粒子半径を変える場合は、
重力・圧力係数・タイムステップも比例して調整しないと同じ挙動になりません
（`Physics/CLAUDE.md`「SPH の長さ単位」）。

### 5. ソルバーの選び方

- **WCSPH** — 素直な弱圧縮性 SPH。このサンプル群の主役で、
  小・大どちらのスケールでも安定に回ります。まずはこれ。
- **PBSPH** — 位置ベース。小スケール＋低 stiffness（`0.001`）の
  検証済み設定なら安定（`03` で使用）。予測位置を壁へ直接クランプするので、
  落ち着くと速度が完全にゼロになります。
- **DFSPH** — 発散フリー。同じ `ISPHSolver` で差し替え可能ですが、
  適応サブステップが敏感で、パラメータの追い込みが要ります
  （現状のデフォルトシーンでは不安定になりやすいため、サンプルでは未使用）。

---

## 次に読むもの

- **`Physics/PhysicsTest/`** — GoogleTest。`WCSPHSolverTest.cpp` /
  `DFSPHSolverTest.cpp` などが、境界・エミッタ・表面張力・Two-Way 結合まで
  含めた「動く最小例」の宝庫です。
- **`Physics/PhysicsView/scenarios/*.json`** — PhysicsView の JSON シナリオ。
  各パラメータの検証済みの値が読めます。
- **`Physics/CLAUDE.md` / `README.md`** — モジュール全体の設計・規約。
- 剛体・軟体・三者結合（Rigid ↔ Fluid ↔ SoftBody）は `PhysicsSolver` が
  入口です（`Physics/Physics/PhysicsSolver.h` のクラスドキュメントコメント参照）。
