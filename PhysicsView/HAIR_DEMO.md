# 長髪・毛皮の共通 API デモ

`scenarios/89_capture_hair_common_api.json` は、同じ HairWorld / HairSolver / HairFollowers を使って
長髪から短毛へ切り替える CPU デモ。外部モデルは不要。
長髪は頭皮上の 32 ガイド＋512 追従毛、短毛はカプセル上の 64 ガイド＋1,024 追従毛を生成する。
どちらも決定的な根元・形状・長さのばらつきを持つ。

## 実行

親リポジトリ `Crystal2024` から、Debug ビルド済みの PhysicsView で実行する。

```powershell
.\Phantom\Physics\PhysicsView\run_physics_scenarios.ps1 -Configuration Debug -Filter 89_capture_hair_common_api
```

PhysicsView を `Phantom/Physics/PhysicsView` を作業ディレクトリとして起動した場合は、
`Window > Scenario Browser` から同名シナリオを選んで実行できる。
シナリオは生成設定を毎回指定するので、同じアプリ内でも再実行できる。

静止形状と、風 2 m/s＋根元アニメーションを固定刻みで 30 ステップ進めた形状を表示・撮影する。
両スタイルでガイドと追従毛の毛先が動いたこと、根元誤差 0、非有限値 0、ガイド長さ誤差 1% 未満を検証する。
Reset の形状復元、teleport の速度初期化、スタイル切替時の時計・風・根元移動の初期化も確認する。

画像は `PhysicsView/testdata/screenshots/89_hair_demo_{long,fur}_{rest,motion}.png` に保存される。
GUI 実行後は短毛を停止して残す。`Window > Command` で `SetHairRunning:true` を実行すると、
同じ固定時間刻みの実時間更新で動かせる。`HairPreset:LongHair` で長髪へ戻せる。
風は `SetHairParam:windX,2` / `SetHairParam:windDrag,1`、根元の動きは `SetHairMotion:true` で設定する。

## C++ からの共通操作

`HairWorld` を一つ所有し、スタイル選択以外は同じ API を使う。戻り値が `false` の場合は更新を中断する。

```cpp
Phantom::HairWorld hair;
if (!hair.setCountParams({32,512,64,1024}) ||
    !hair.setGenerationParams({0.5f,0.02f,0.05f,2026}) ||
    !hair.setPreset(Phantom::HairPreset::LongHair)) return;
auto params = hair.params();
params.windVelocity = {2.f,0.f,0.f};
params.windDrag = 1.f;
if (!hair.setParams(params)) return;
hair.setMotion(true);
hair.setRunning(true);
// 各描画フレームで elapsedSeconds を渡す。false は更新なしも意味する。
hair.update(elapsedSeconds);
auto lines = hair.buildWireData(); // ガイドと補間した追従毛の描画データ
// スタイル切替では時計・風・根元移動が初期化され、停止する。
if (!hair.setPreset(Phantom::HairPreset::ShortFur)) return;
```

ガイドだけを物理計算し、追従毛は補間する。線表示で、追従毛自体の長さ制約・衝突はない。
頭皮・カプセルは生成面であり、このデモでは身体コライダーを登録しない。
GPU 化・大量本数の性能測定は後続 Phase の対象。
