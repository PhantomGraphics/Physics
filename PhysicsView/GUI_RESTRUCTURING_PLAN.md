# PhysicsView GUI 構造改良計画

## 1. 目的

`PhysicsView` の GUI を整理し、操作対象を見つけやすく、機能追加時にもパネル構成が肥大化しにくい構造へ改良する。

GUI の構成と操作フローについては `ref/FluidView` を参考にする。特に、メニューから作業対象を選び、共通の Control 領域へ対応する操作画面を表示する構造を取り入れる。

本計画で参考にするのは GUI 構造のみである。`ref/FluidView` の `Scene`、`Presenter`、`Animator` などのアプリケーション構造や描画方式は移植しない。

## 2. 対象範囲

### 2.1 変更対象

- `PhysicsView` のメインメニュー
- GUI パネルの選択方法と配置
- 既存パネルを共通 Control 領域へ埋め込むためのインターフェース
- Fluid 操作画面内の項目整理
- GUI 上のシミュレーション状態表示
- GUI の表示状態保存
- 必要な場合に限り、`CGLib/UIWidgets` および `VkAppBase` の GUI 関連 API

### 2.2 原則として変更しないもの

- `FluidWorld`、`RigidBodyWorld`、`SoftBodyWorld` のシミュレーション構造
- `PhysicsSolver` および各 SPH ソルバー
- Rigid-Fluid / SoftBody-Fluid 連成処理
- Vulkan Renderer と GPU バッファ同期の構造
- JSON シナリオの形式とコマンド仕様
- `CGLib/Scene` の `Scene`、`Presenter`、`Animator` 構造

GUI の整理に必要な小規模な呼び出し口の追加は許容するが、物理処理の責務分割やアルゴリズム変更は本計画に含めない。

## 3. 現状と課題

現在の `FluidApp` は、複数の `IVkUIPanel` を直接登録し、メインメニューの `View` から各ウィンドウの表示を個別に切り替えている。

対象となる主なパネルは次のとおりである。

- Fluid Control
- Rigid Body Control
- Soft Body Control
- Fluid Renderer
- SSFR Control
- SSFR Test
- Volume Conversion
- Scenario Browser

この方式には次の課題がある。

- 複数の操作ウィンドウが同時に開き、画面が散らかりやすい
- 目的の機能が `View` メニュー内に平坦に並び、機能の分類が分かりにくい
- パネルごとにウィンドウ生成と内容描画が一体化しており、別レイアウトへ再利用しにくい
- Fluid Control の設定項目が増え、シミュレーション、物性、境界、Emitter、Outflow が一画面に集中している
- 別の操作ページへ移ると、実行状態や粒子数などの重要情報を確認しにくい

## 4. 参考にする GUI 構造

`ref/FluidView` では `PhysicsMenu` が処理の種類を列挙し、選択された View を共通の `Control` パネルへ設定している。

この考え方を現在の ImGui + Vulkan GUI に合わせ、次の構造として導入する。

```text
Main menu
 ├─ File
 ├─ Camera
 ├─ Physics
 │   ├─ Fluid
 │   ├─ Rigid Body
 │   ├─ Soft Body
 │   ├─ Fluid Rendering
 │   ├─ SSFR
 │   ├─ Volume / Mesh Conversion
 │   ├─ Scenario Browser
 │   └─ SSFR Test
 └─ View
     └─ Control Window

Control Window
 ├─ 共通ステータス
 ├─ 共通実行操作
 └─ 選択中の操作ページ
```

`Physics` メニューは操作ページを選択する。`View` メニューは Control ウィンドウ自体の表示・非表示など、レイアウトに関する操作だけを担当する。

## 5. 目標設計

### 5.1 ControlPage

表示する操作ページを型として管理する。

```cpp
enum class ControlPage {
    Fluid,
    RigidBody,
    SoftBody,
    FluidRendering,
    SSFR,
    VolumeConversion,
    ScenarioBrowser,
    SSFRTest,
};
```

文字列や複数の `visible` フラグで選択状態を管理せず、原則としてアクティブなページを一つに限定する。

### 5.2 ControlPanelHost

共通 Control ウィンドウを管理する `ControlPanelHost` を追加する。

責務は次のとおりである。

- 現在選択されている `ControlPage` の保持
- ページ名と埋め込みパネルの対応付け
- Control ウィンドウの生成
- 共通ステータスと実行操作の表示
- 選択中パネルの内容描画
- 未登録ページや利用不能ページの安全な表示

想定インターフェースは次のとおりである。

```cpp
class ControlPanelHost : public VKG::IVkUIPanel {
public:
    void registerPage(ControlPage page, IEmbeddedPanel& panel);
    void setPage(ControlPage page);
    ControlPage getPage() const;

    void setVisible(bool visible);
    bool isVisible() const;

    void onImGui() override;
};
```

ページの所有権は従来どおり `FluidApp` 側に置く。`ControlPanelHost` は非所有参照だけを保持し、既存オブジェクトの寿命を変えない。

### 5.3 埋め込み可能なパネル

各パネルについて、ウィンドウ生成と内容描画を分離する。

```cpp
class IEmbeddedPanel {
public:
    virtual ~IEmbeddedPanel() = default;
    virtual void drawContents() = 0;
};
```

各パネルは `drawContents()` に実際のコントロールを描画する。既存の独立ウィンドウ表示を互換性のため残す場合、`onImGui()` はウィンドウを開始して `drawContents()` を呼ぶだけにする。

```cpp
void SomePanel::onImGui()
{
    if (!visible_) return;
    if (ImGui::Begin("Some Panel", &visible_)) {
        drawContents();
    }
    ImGui::End();
}
```

PhysicsView 専用のインターフェースとして開始し、他のアプリでも有用だと確認できた場合にだけ CGLib へ移す。

### 5.4 PhysicsMenu

メインメニュー内に `Physics` メニューを追加する。

メニュー項目を選択したときは次の二つを行う。

1. `ControlPanelHost` のアクティブページを変更する
2. Control ウィンドウが非表示なら表示する

選択中のページにはチェック表示を付ける。シミュレーション方式などによって使用できないページは disabled 表示とし、理由をツールチップで説明する。

## 6. GUI ページ構成

### 6.1 Fluid

現在の Fluid Control を次のセクションへ整理する。

```text
Fluid
 ├─ Simulation
 │   ├─ Method
 │   ├─ Time Step
 │   └─ Run / Pause / Step / Reset
 ├─ Material
 │   ├─ Density
 │   ├─ Viscosity
 │   ├─ Pressure Coefficient / Stiffness
 │   └─ Surface Tension
 ├─ Initial Region
 │   ├─ Particle Radius
 │   ├─ Effect Length
 │   └─ Fluid Bounds
 ├─ Boundaries
 │   ├─ Simulation Boundary
 │   ├─ Boundary Damping
 │   └─ Mesh Boundary
 ├─ Emitters
 └─ Outflow Regions
```

方式によって意味を持たない設定は非表示または disabled にする。disabled にする場合は、使用できない理由を同じ場所で確認できるようにする。

### 6.2 Rigid Body

- プリセット選択
- Run / Pause / Step / Reset
- 剛体追加操作
- ソルバーパラメーター
- Fluid coupling の設定と対応状況

### 6.3 Soft Body

- プリセット選択
- Run / Pause / Step / Reset
- クロス、ロープ、ゼリー関連設定
- 自己衝突・物体間衝突設定
- Fluid coupling の設定と対応状況

### 6.4 Fluid Rendering

- Point renderer の有効化
- 密度カラー範囲
- 自動レンジ
- 粒子表示サイズなどの表示専用設定

シミュレーションパラメーターと描画パラメーターを同じページに混在させない。

### 6.5 SSFR

- SSFR の有効化
- 表示モード
- Thickness、Filter、Reflection、Refraction 設定
- Environment map 関連設定

Point renderer と SSFR の排他的な表示関係は、現在の動作を維持する。

### 6.6 Volume / Mesh Conversion

- Fluid から Volume への変換
- Volume から Mesh への変換
- 各変換パラメーター
- Volume / Mesh の表示切り替え
- 変換結果とエラーの表示

### 6.7 Scenario Browser

既存の `ScenarioBrowserPanel` を共通 Control ウィンドウへ埋め込む。

ライブラリ側のパネルが独立ウィンドウしか描画できない場合は、後方互換性を維持したまま `drawContents()` 相当を追加する。

### 6.8 SSFR Test

通常操作から明確に区別できるよう、メニュー末尾へ配置する。ページ見出しにもテスト用であることを表示する。

## 7. 共通ステータスと操作

Control ウィンドウ上部に、ページを移動しても常に確認できるステータス領域を設ける。

表示候補:

- 現在の流体方式
- Running / Paused
- Fluid、Rigid Body、Soft Body の個別実行状態
- 粒子数
- Rigid-Fluid / SoftBody-Fluid coupling の状態
- シナリオ実行状態
- 直近の警告またはエラー

表示例:

```text
DFSPH | Fluid: Running | Rigid: Paused | 8,000 particles | Coupling: Two-way
```

Run / Pause、Step、Reset を完全に共通化すると各ドメインの独立制御を損なう可能性があるため、最初は Fluid の共通操作だけを表示するか、操作対象を明示する。曖昧な一括 Reset は追加しない。

## 8. 実装フェーズ

### Phase 0: GUI 動作の基準固定

- 現在の全パネルと操作項目を一覧化する
- 各操作が呼び出す World / Renderer のメソッドを記録する
- GUI 改修前後で操作の欠落がないことを確認するチェックリストを作る
- 代表シナリオを実行し、GUI改修が物理結果へ影響しない基準を確保する

完了条件:

- 既存操作の対応表がある
- GUI変更後に確認する代表シナリオが決まっている

### Phase 1: ControlPanelHost の追加

- `ControlPage` を追加する
- `ControlPanelHost` を追加する
- 既存パネルの所有権を変えずに登録できるようにする
- 最初は一つの既存パネルだけを埋め込み、ライフサイクルを確認する

完了条件:

- Control ウィンドウ内に選択した一つのパネルを表示できる
- Vulkan初期化・終了順序が変わらない

### Phase 2: 既存パネルの埋め込み対応

- 各パネルの `onImGui()` から内容描画を `drawContents()` へ抽出する
- 全パネルを `ControlPanelHost` に登録する
- 同じパネルが独立ウィンドウとControl領域の両方で二重描画されないようにする

完了条件:

- 全既存パネルを共通Control領域に表示できる
- すべての既存操作が使用できる

### Phase 3: PhysicsMenu の追加

- `Physics` メニューを追加する
- メニュー選択と `ControlPage` を接続する
- 選択状態、disabled状態、ツールチップを追加する
- `View` メニューをウィンドウ表示管理へ限定する

完了条件:

- メニューから一操作で目的のページへ移動できる
- 選択中のページが明確に分かる

### Phase 4: Fluid ページの整理

- Fluid Control をセクション分割する
- ソルバー方式に応じて項目の表示状態を整理する
- 危険または大きな再生成を伴う操作を視覚的に区別する
- 単位、値域、反映タイミングをラベルまたはツールチップで示す

完了条件:

- Fluid の主要操作へ少ないスクロールで到達できる
- 現在の方式に無効な設定を誤操作しにくい

### Phase 5: 共通ステータスの追加

- 実行状態と粒子数を表示する
- coupling とシナリオの状態を表示する
- エラー表示の位置と寿命を統一する

完了条件:

- どのページからでもシミュレーション状態を確認できる
- エラーが発生した操作と内容をGUI上で識別できる

### Phase 6: 表示状態の保存と仕上げ

- 最後に選択したページを保存する
- Control ウィンドウの位置とサイズを保存する
- セクションの開閉状態を保存する
- 初期ウィンドウサイズでレイアウト崩れがないことを確認する
- 高DPIとウィンドウリサイズを確認する

完了条件:

- 再起動後に自然な作業状態へ戻れる
- 1280×720を基準に、主要操作が無理なく行える

## 9. ライブラリ側変更の判断基準

最初から共通ライブラリへ抽象化せず、PhysicsView 内で設計を検証する。

次の条件を満たす場合にだけ CGLib 側へ移す。

- PhysicsView 以外にも同じ埋め込みパネル機構を必要とする利用者がある
- アプリ固有の `ControlPage` や物理型に依存しない
- 既存の `IVkUIPanel` 利用者を変更せずに導入できる
- 所有権と `onImGui()` の呼び出し回数が明確である

想定されるライブラリ変更は GUI に限定する。

- 埋め込み表示用インターフェース
- disabled scope
- tooltip helper
- 共通パネルホスト
- `ScenarioBrowserPanel` の内容描画分離

`CGLib/Scene` と物理ライブラリには、本GUI改修を理由とした変更を加えない。

## 10. テスト計画

### 10.1 GUI確認

- 全メニュー項目から正しいページへ移動できる
- Control ウィンドウを閉じ、メニューから再表示できる
- ページ切り替え後も入力値が保持される
- パネルが一フレームに複数回描画されない
- disabled 項目を操作できない
- 小さいウィンドウでも操作不能にならない

### 10.2 機能回帰

- Fluid の方式変更、Reset、Run、Pause、Step
- Rigid Body と Soft Body のプリセット変更
- Rigid-Fluid / SoftBody-Fluid coupling
- Emitter と Outflow
- Mesh boundary
- Point renderer と SSFR の切り替え
- Volume / Mesh conversion
- Scenario Browserからのシナリオ実行
- Screenshotシナリオ

### 10.3 非回帰条件

- JSON シナリオファイルを変更しない
- シナリオコマンド名と応答を変更しない
- 同じ初期条件とステップ数で物理結果が変わらない
- GPU CSPH と SSFR のバッファ同期順序が変わらない
- Vulkan cleanup の順序が変わらない

## 11. リスクと対策

### パネルの二重描画

同じパネルを `VkAppBase` と `ControlPanelHost` の両方が描画すると、操作が二重に適用される可能性がある。

対策:

- 埋め込み後はパネル本体を `VkAppBase::add()` のUI描画対象にしない
- またはパネルを一箇所からだけ呼ぶ所有・登録ルールを設ける

### `onImGui()` と `drawContents()` の責務混在

ウィンドウ開始・終了が内容描画に残ると、埋め込み時に ImGui のスタックが崩れる。

対策:

- `drawContents()` では `Begin` / `End` を呼ばない規約を明記する
- 内容描画単体の確認を行う

### GUI変更による物理操作の変化

ボタンの配置変更時に、Reset前提の値や即時反映値の扱いを変えてしまう可能性がある。

対策:

- 各項目の反映タイミングを変更前に記録する
- 初期フェーズでは既存コールバックをそのまま再利用する
- 操作仕様の改善はGUI構造変更とは別の変更として扱う

### 共通化の過剰化

PhysicsView 固有の要件を早期に CGLib へ入れると、他アプリに不要な抽象化が残る。

対策:

- `ControlPanelHost` と `ControlPage` はまず PhysicsView 内へ実装する
- 二つ目の実利用が確認できてから共通化する

## 12. 推奨する最初の実装単位

最初の変更は次の範囲に限定する。

1. `ControlPage` と `ControlPanelHost` を追加する
2. Fluid Control を `drawContents()` 対応にする
3. `Physics` メニューの `Fluid` 選択から表示する
4. 従来の Fluid 操作がすべて動作することを確認する
5. 代表的な Fluid シナリオを実行する

この単位で GUI コンテナ設計と ImGui ライフサイクルを検証した後、Rigid Body、Soft Body、SSFR、Conversion、Scenario Browser の順に移行する。

## 13. 最終完了条件

- GUIが「メニューによる機能選択 + 共通Control領域」の構造になっている
- 選択中の操作ページだけがControl領域に表示される
- Fluid、Rigid Body、Soft Body、Rendering、SSFR、Conversion、Scenarioの全既存操作が利用できる
- シミュレーション状態をどのページからでも確認できる
- GUI変更を理由とした物理・Scene・Renderer構造の改変がない
- 既存シナリオの仕様と結果が維持される
- GUI固有の共通化だけが、必要性を確認したうえでCGLibへ追加される
