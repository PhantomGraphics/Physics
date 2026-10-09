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
GPU シミュレーションはユーザー指定でスキップ。大量本数の性能測定は後続項目。

## CPU 版 LOD

`SetHairLodParam:enabled,1` で、カメラから根元リグ中心までの距離による自動 LOD を有効にする。
既定は無効なので、上のデモの本数・固定刻みを保つ。

| 段階 | 距離境界（既定） | 長髪の粒子数 / 本 | 短毛の粒子数 / 本 | 実時間更新 | 描画する追従毛 |
|---|---|---|---|---|---|
| 0 | 近距離 | 24 | 4 | 既定 60 Hz | 全本数 |
| 1 | 6 | 13 | 3 | 既定 30 Hz | 2 本に 1 本 |
| 2 | 12 | 7 | 2 | 既定 15 Hz | 4 本に 1 本 |

`SetHairLodParam:mediumDistance,6` / `farDistance,12` / `hysteresis,0.5` で境界を調整する。
遠方へ進むと境界＋hysteresis、近づくと境界−hysteresis で切り替える。
既存カメラの最小距離は 5 なので、近距離へ戻せるよう mediumDistance−hysteresis を 5 より大きくする。
`GetHairLodStat:level` / `distance` / `updateScale` / `drawnFollowers` で現在値を取得できる。

粒子は元の静止形状と現在の静止形状からの変位・速度を再サンプルして切り替える。
根元・毛先・進行時間・ステップ数を保持し、近距離への復帰時は元の静止形状の曲率を再利用する。
追従毛の補間頂点は保持し、描画時だけ本数と頂点を間引く。元の生成本数設定は変更しない。
Reset は現在の LOD の静止形状へ戻り、LOD 無効化は元の粒子数へ戻る。

更新を間引く際は時間刻みを 2 / 4 倍にして実時間を保持する。
ソルバーの上限（1/15 s）を超える場合は倍率を下げる。手動 `HairStep` は常に設定済みの基準刻みを 1 回進める。
既存の追い付き上限と破棄時間の記録は基準刻みのまま。
物理粒子数は即時に切り替え、表示形状と追従毛の透明度は既定 0.15 秒で補間する。
`SetHairLodParam:transitionSeconds,0.15` で 0～2 秒の遷移時間を設定でき、0 は即時切替。
`GetHairLodStat:transitionProgress` は 0（開始）～1（完了）を返す。
物理計算の停止中も実時間で表示遷移を進める。手動 Step は基準刻み分だけ表示遷移も進める。
`GetHairLodStat:drawnFollowers` は切替先の定常描画本数で、遷移中は消える毛も一時的に描画する。

遷移中は全 LOD の頂点配置を含む共通ポリラインで補間し、完了時に不要な頂点を外す。
逆方向へ切り替えた場合は現在表示している形状・透明度から再開する。スナップショットが増え続けることはない。
動く根元に補間形状を追従させ、Reset / teleport / Clear では古い表示を破棄する。
透明度は髪専用の線描画でブレンドし、透明な線は深度を書き込まない。
透明線の交差は描画順に依存する近似で、厳密な透過処理は後続の描画品質項目に含む。
細部の完全な復元や、更新・補間・描画の性能計測は後続項目。

`09_smoke_hair_lod` がカメラによる切替・元の粒子数への復帰・時計保持・不正設定を検証する。
`09_smoke_hair_lod_transition` は風を加えた形状の遠近切替・表示遷移の完了・時計保持・teleport を検証し、
`testdata/screenshots/09_hair_transition_{out,in}.png` を保存する。
