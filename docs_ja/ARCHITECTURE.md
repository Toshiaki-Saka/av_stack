# アーキテクチャとモジュール解説

> 英語版: [`../docs_en/ARCHITECTURE.md`](../docs_en/ARCHITECTURE.md)

各構成要素が *どのように* 動作し、*なぜ* そのように設計されているのかを理解したい読者向けの、コードベースのガイドツアーである。本スタックは **C++ ファースト** であり、コントローラ、IMM トラッカー、センサ、パーセプション、プランニング、セーフティガードレール、オキュパンシーといったパイプラインのアルゴリズムはすべて `cpp/include/` 配下の C++17 ヘッダとして実装され、pybind11 経由で Python に公開される（`*_cpp` モジュール）。`python/*.py` レイヤは可視化・デモ・シナリオドライバのフロントエンドであり、加えて議論全体を束ねるセーフティケースを担う。

```
            ┌──────────────────── safety case (GSN) ─────────────────────────┐
            │  SAFETY_CASE.md · safety/guardrail.hpp · run_pipeline.py        │
   ┌────────┴── safety monitor ─────────────────────────────────────────────┐│
   │  safety/guardrail.hpp  (RSS long + TTC + RSS lat)                       ││
   │ ┌──── planning & perception (C++) ───────────────────────────────────┐ ││
   │ │  planning/*.hpp · perception/fusion.hpp · world/occupancy.hpp ·     │ ││
   │ │  sensors/sensors.hpp                                                │ ││
   │ │ ┌──── C++ numerics core ────────────────────────────────────────┐  │ ││
   │ │ │ control/bicycle.hpp · lqr.hpp · mpc.hpp · world/tracking.hpp │  │ ││
   │ │ └──────────────────────────────────────────────────────────────┘  │ ││
   │ └────────────────────────────────────────────────────────────────────┘ ││
   └──────────────────────────────────────────────────────────────────────────┘│
   pybind11 → *_cpp modules · python/ frontend: animation_demo.py · run_*.py    │
```

上図のボックスはすべて C++（`cpp/include/**.hpp`）であり、pybind11 の `*_cpp` モジュールを介して Python から利用される。以下では各モジュールを対応する C++ ヘッダに紐づけて解説する。

---

## 1. C++ 数値計算コア

### `cpp/include/control/bicycle.hpp` — キネマティック自転車モデル

キネマティック自転車モデルは、予測とシミュレーションの両方で全体を通じて用いられるモデルである。状態は $state = [x, y, \psi, v]$、入力は $input = [a, \delta]$ である。

- **設計判断 — Euler ではなく RK4。** $dt = 0.1 s$ かつ高速道路速度域では、Euler 積分は 1 秒あたり約 0.5 m/s の誤差を蓄積する。RK4 ではこれが無視できる水準まで低下する。MPC の予測精度はこの点に直接依存する。
- **設計判断 — 有限差分によるヤコビアン。** RK4 の連鎖律を解析的に導出する方法（誤りやすく、ステップを変更するたびに再導出が必要）ではなく、中心差分 $A(i,j) = (f(s+\varepsilon e_j)-f(s-\varepsilon e_j))/(2\varepsilon)$ を用いる。これにより、非線形ステップと *証明可能な意味で整合する* ヤコビアンが得られる。
- **検証。** `control_test` は線形化残差 $`\lVert step(s_0+\delta s, u_0) - (step(s_0,u_0) + A\,\delta s) \rVert`$ が $O(\varepsilon^2)$ であること（残差 $2.3\times10^{-6}$）を確認する。これはヤコビアンが非線形ステップと 2 次の精度で一致していることを裏づける。

### `cpp/include/control/linalg.hpp` — 密行列

ヘッダオンリーの最小限の密行列クラス（行優先、`double`）である。`operator*`、転置 `T()`、Gauss-Jordan 法による `solve(A, B)` を提供する（部分ピボット選択は行わない。ここで扱う行列は最大でも $30\times30$ であり、常に良条件だからである）。外部 BLAS への依存はなく、C++17 コンパイラが存在する環境ならどこでもビルドできる。

### `cpp/include/control/lqr.hpp` — PID と LQR

**PID。** 積分クランプ（`[i_min, i_max]` によるアンチワインドアップ）と出力飽和を備えた標準的な比例・積分・微分制御である。縦方向の速度維持に用いるほか、`animation_demo.py` では横方向のレーンキーピングにも用いる。

**`dlqr`。** 後退 **DARE 反復** により離散 LQR ゲイン `K` を求める。

```math
\begin{aligned}
P &\leftarrow Q + A^\top P A - A^\top P B (R + B^\top P B)^{-1} B^\top P A \quad \text{(until } \lVert \Delta P \rVert < tol) \\
K &= (R + B^\top P B)^{-1} B^\top P A
\end{aligned}
```

解の近傍では収束は 2 次であり、`iters = 1000` と $tol = 1\times10^{-10}$ は、ここで扱う 2 状態モデルには十分すぎる設定である。

**`LateralLQR`。** 速度 $v$ における 2 状態誤差モデル $[e_y, e_\psi]$ に対して `dlqr` をラップしたものである。参照曲率を打ち消すフィードフォワード $\delta_{ff} = \arctan(L \kappa)$ を併用する。速度が線形化に入るため、ゲインは毎ステップ再計算される（ゲインスケジューリングの LTV 版に相当する）。**検証:** `test_controllers_track` は自車をダブルレーンチェンジに沿って走行させる。LQR の定常横偏差は $8.6\times10^{-14}$ m である。

### `cpp/include/control/mpc.hpp` — 線形時変 MPC

**問題設定。** アクチュエータ制約 $a \in [a_{min}, a_{max}]$、 $\delta \in [\delta_{min}, \delta_{max}]$ のもとで、 $k = 0 \dots N$ にわたる参照軌道 $(s_{ref,k}, u_{ref,k})$ を追従する。

**線形化。** 各ステージで RK4 ステップを線形化して $(A_k, B_k)$ とアフィン欠損項 $d_k = step(s_{ref,k}, u_{ref,k}) - s_{ref,k+1}$ を得る。誤差座標 $e_k = s_k - s_{ref,k}$ では $e_{k+1} = A_k e_k + B_k \Delta u_k + d_k$ となる。

**コンデンシング。** 前向きに再帰することで $E = S_x e_0 + S_u \Delta U + offset$ が得られ、QP は次の形になる。

```math
\min \tfrac{1}{2} \Delta U^\top H \Delta U + g^\top \Delta U \quad \text{s.t.} \quad lo \le \Delta U \le hi, \qquad H = S_u^\top \bar{Q} S_u + \bar{R}
```

**設計判断 — 内点法ではなく FISTA。** コンデンシング後の QP は常に強凸（H は正定）で、かつボックス制約のみである。これは FISTA にとって理想的なケースであり、射影が閉形式で求まり、1 反復あたり $O(N_m N_n)$、LP ソルバへの依存もない。Lipschitz 定数は H に対する 60 ステップの **べき乗反復** で推定する。ここで用いるホライズンと重みに対しては、200 回の FISTA 反復で確実に収束する。

**検証。** `test_cpp_control_test` は `control_test.exe` を呼び出す。
- 速度追従: $v = 12.00$ m/s、 $\max|a| = 3.00$ m/s²（制限は 3.0）✓
- 横方向復帰: $y = 0.000$ m、 $\max|\delta| = 0.600$ rad（制限は 0.6）✓

### `cpp/include/world/tracking.hpp` — 多物体 IMM トラッカー

物体ごとに **IMM（Interacting Multiple Model）** フィルタ（`struct IMM`）を割り当て、プロセスノイズの異なる 2 つの等速度（CV）カルマンフィルタ、すなわち低ノイズの *巡航* モデル（$`q = 0.5`$）と高ノイズの *機動* モデル（$`q = 6.0`$）を混合する。状態は $[x, y, v_x, v_y]$ であり、各構成フィルタは標準的な CV の予測／更新を実行する。

```math
\begin{aligned}
\text{Predict:} \quad & \hat{x} = F x, \quad \hat{P} = F P F^\top + Q \quad \text{(} F = I + dt\cdot block,\ Q = \text{process noise)} \\
\text{Update:} \quad & K = \hat{P} H^\top (H \hat{P} H^\top + R)^{-1}, \quad x \leftarrow \hat{x} + K(z - H \hat{x}), \quad P \leftarrow (I - KH) \hat{P}
\end{aligned}
```

**モード混合。** 各ステップで IMM は 2 つのモデルを相互作用させ（マルコフ遷移確率に基づいて状態を混合し）、両フィルタを実行したうえで、各モデルの観測尤度からモード確率 $\mu$ を更新し、 $\mu$ で重み付けした混合として統合推定値を構成する。`p_manoeuvre()` は機動モードの確率を公開する。これにより、対象が巡航している間は推定を締まった状態に保ちつつ、制動やカットインが起きたときには素早く反応できる。ノイズを固定した単一の CV フィルタより優れる点である。

**データアソシエーション。** マハラノビスゲート付きの最近傍法である。マッチしなかった検出は暫定トラックを初期化する。トラックはクラッタを抑制するため、`min_hits` 回の一貫した更新を経て初めて *確定* される。

**プランナ向け予測。** `predict(k)` は確定済みの各トラックを等速度モデルで `k` ステップ前方伝播し、`(x, y)` のリストを返す。プランナはこれを用いて回避コストを計算する。

**検証。** `test_tracking`: 3 秒のコールドスタート後、トラッカーは先行車を平均位置誤差 0.6 m 未満、速度誤差 0.4 m/s 未満で維持する。

---

## 2. センサモデル (`cpp/include/sensors/sensors.hpp` → `sensors_cpp`)

物体レベルのセンサモデルを 3 種類用意している。各モデルはワールドが返す `objects = [(id, state, kind)]` を入力とし、可視性（レンジ + 半 FOV）を判定し、見逃し確率を適用して、観測値 `z` と共分散 `R` を持つ `Detection` オブジェクトを返す。

| センサ | ノイズモデル | R の形 |
|---|---|---|
| **LiDAR** | $[x, y]$ に対する等方ガウスノイズ $\sigma = 0.15$ m | $\sigma^2 I_2$ |
| **Radar** | レンジノイズ $\sigma_r = 0.4$ m、横方向ノイズ $\sigma_{lat} = 1.2$ m、LOS 角で回転 | $\mathrm{Rot} \cdot \mathrm{diag}(\sigma_r^2, \sigma_{lat}^2) \cdot \mathrm{Rot}^\top$ |
| **Camera** | 方位ノイズ $\sigma = 0.6^\circ$、レンジ相対誤差 10 %、LOS で回転 | $\mathrm{Rot} \cdot \mathrm{diag}((r\cdot err)^2, (r\cdot\sigma_b)^2) \cdot \mathrm{Rot}^\top$ |

**設計判断 — Radar / Camera に異方性の R を用いる。** ユークリッド距離によるゲーティングはカメラでは破綻する（レンジ誤差が数メートルに達しうるため）。マハラノビス距離に $(R_i + R_j)$ を用いることで、センサの方向依存な不確かさを正しく重み付けできる。

---

## 3. パーセプション / センサフュージョン (`cpp/include/perception/fusion.hpp` → `perception_cpp`)

**クラスタリング。** 貪欲法によるアソシエーションである。未対応の検出それぞれについて、マハラノビス距離 $`\sqrt{d^2} \lt \sqrt{9.21}`$（$`\chi^2`$ ゲート、自由度 2、99 %）以内にある検出をすべて集める。

**フュージョン。** 各クラスタ内で、情報形式により統合する。

```math
R_{fused} = \left( \sum_i R_i^{-1} \right)^{-1}, \qquad z_{fused} = R_{fused} \cdot \sum_i R_i^{-1} z_i
```

これはセンサ間の独立性を仮定した最尤推定である。統合後の共分散は、常に個々のセンサ単独の共分散より締まったものになる。

**速度の事前情報。** クラスタ内に Radar の検出が含まれる場合、その Doppler 計測値を推定 LOS 方位へ射影して $[v_x, v_y]$ を構成し、新規トラックのウォームスタートに用いる。これによりカルマンフィルタのコールドスタート時の速度誤差を回避できる。

---

## 4. 軌道とパス (`cpp/include/planning/{trajectory,path}.hpp` → `planning_cpp`)

**`Trajectory`。** $(x[k], y[k], v[k])$ を保持し、数値微分によって方位 $\psi[k]$ と曲率 $\kappa[k]$ を導出する。プランナの出力かつ MPC の参照として用いられる。

**`Path`。** `smoothstep(x, a, b)` を用いて前進距離でパラメータ化したダブルレーンチェンジである。以下を提供する。
- `errors(state)` — 符号付き横偏差 $e_y$、方位誤差 $e_\psi$、局所曲率 $\kappa$、 $v_{ref}$。
- `mpc_window(state, N)` — MPC 用のフラット配列 $(s_{ref}, u_{ref})$。各ステージでフィードフォワード $u_{ff} = [\Delta v / dt, \arctan(L \kappa)]$ を含む。

---

## 5. プランナ (`cpp/include/planning/planner.hpp` → `planning_cpp`)

プランナは、古典的な **挙動決定と軌道生成を分離した（decoupled behaviour/trajectory）** 構造をとる。

1. **候補生成。** 候補レーン（自車レーン、左レーン）ごとに `_gen(ego, target_lane, preds)` を呼ぶ。
   - 縦方向: 同一レーン内で最も近い予測先行車に対する IDM。先行車がなければフリーロード IDM。
   - 横方向: 現在の `y` から `target_lane` へ `t_change` かけて smoothstep で遷移。

2. **安全性による棄却。** `_cost` は、予測された全エージェント × 軌道の全ステップにわたる最小クリアランスを計算する。`min_clear < safe_radius` であれば、その候補は `None`（棄却）となる。これはプランナ自身が持つ衝突回避である。

3. **スコアリング。** 受理された候補は次のように評価される。

```math
\begin{aligned}
   cost = {} & \text{speed-keeping} + (-\text{progress}) + \text{comfort}(\max v^2\kappa) + \text{lane-change penalty} \\
   & + \text{lane-preference} + 30/\text{min\_clear}
   \end{aligned}
```

   ソフト項 $`30/\text{min\_clear}`$ は、余裕が大きい段階では候補をハード棄却することなく、プランナを近接エージェントから遠ざける働きをする。

4. **フォールバック。** すべての候補が棄却された場合、プランナは現在レーンで `1.5b` の減速を行い、`EMERGENCY_SLOW` を返す。実際の制動はその後ガードレールが引き継ぐ。

**`fault` モード。** `fault=True` を設定すると、劣化したプランナチャネルを模擬する（極端に小さい車間時間 `T = 0.2 s`、最小限の制動 `b = 1.0`、`safe_radius = 0`）。これは `run_pipeline.py` におけるシナリオであり、ガードレールが主チャネルの故障を捕捉する様子を示すためのものである。

---

## 6. ガードレール (`cpp/include/safety/guardrail.hpp` → `guardrail_cpp`)

ガードレールは機能安全上もっとも重要な構成要素であると同時に、もっとも単純な構成要素でもある。これは意図的な設計である。セーフティモニタは *独立に実装・検証可能* でなければならず、それ自体がさらに監視を必要とするような複雑なモジュールであってはならない。

### 縦方向 RSS

```math
d_{RSS}(v_{ego}, v_{lead}) = v_{ego}\cdot\rho + \tfrac{1}{2}a\cdot\rho^2 + \frac{(v_{ego} + \rho a)^2}{2b} - \frac{v_{lead}^2}{2b_{lead}}
```

パラメータ: $\rho = 0.4$ s（反応時間）、 $a = 1.0$ m/s²（$`\rho`$ の間の自車最大加速度）、 $b = 4.0$ m/s²（自車の最小制動能力）、 $b_{lead} = 8.0$ m/s²（先行車の最大制動）。

この式は、先行車が最大制動をかけ、かつ自車の反応に $\rho$ 秒（その間さらに加速する可能性がある）を要したとしても、その後に自車が最小制動能力で減速すれば衝突が起きない *最小車間* を与える。

### TTC チェック

```math
TTC = gap / (v_{ego} - v_{lead}) \quad \text{if } v_{ego} > v_{lead}
```

$`TTC \lt 2.5`$ s のときにフラグを立てる。

### 横方向 RSS

危険なカットインを、次の 2 条件が同時に成立するかどうかで検出する。
1. 横方向のギャップ $`\lt \mu + \text{travel}(v_{vy,other}) + \text{travel}(v_{vy,ego})`$（横方向 RSS 距離）。
2. 縦方向のギャップが RSS のバンド内にある（上記と同じ式）。

RSS モデルによれば、状況が危険と判定されるのは *両方の* 安全距離が侵害されたときのみである。そして（安全な横方向の回避が取れない場合の）正しい応答は、縦方向に制動することである。

### ラッチ

いずれかのチェックが発火すると `self.latch` が `hold`（デフォルト 8 ステップ = 0.8 s）に設定され、各ステップでデクリメントされる。ガードレールはラッチが切れるまで $-b_{emergency} = -6$ m/s² を代入し、（安定性のため）最後の操舵指令を保持する。

**ラッチの設計根拠。** RSS 条件が解消した瞬間にオフになるガードレールは、境界上で振動してしまう。ラッチは、プランナに権限を返す前に、車両が実際に安全な状態まで減速していることを保証する。

---

## 7. オキュパンシーグリッド (`cpp/include/world/occupancy.hpp` → `occupancy_cpp`)

プランナとガードレールに対して、前方道路の確率的なビューを提供する。

予測された各エージェントの軌道は、**車両フットプリント形状のガウス分布** としてスプラットされる。すなわち、エージェント中心の 2 次元ガウス分布（$`\sigma(k) = \sigma_0 + growth\cdot k`$、予測ホライズンとともに不確かさが膨らむ）を、 $car_l \times car_w$ の矩形フットプリントで畳み込む。エージェント間では次のように統合する。

```math
P(\text{cell occupied}) = 1 - \prod_i (1 - P_i(\text{cell})) \quad \text{(probabilistic OR)}
```

これにより、滑らかで微分可能なマップが得られる。プランナの候補軌道に沿ってオキュパンシーを積分すればソフトコストとして使えるし、しきい値を超えるセルを通過する軌道をブロックすればハードな拒否条件としても使える。

---

## 8. セーフティケース

`SAFETY_CASE.md` は、以上の要素を **Goal Structuring Notation** による保証議論として組み立てる。トップレベルの安全目標、それをノミナル性能 / ガードレールによる故障緩和 / 有界な残留リスクへと分解するストラテジ、そして本リポジトリ内で再現可能な結果を指し示すリーフのソリューション群から構成される。

議論の誠実な構造は次のとおりである。プランナは **ノミナル性能チャネル** であって、安全機構ではない。安全性はもっぱら独立したガードレールが担保する。残留リスクは（RSS パラメータの校正誤差がそのまま車間距離の誤差に直結する、というかたちで）隠さずに明示している。

---

## すべてを再現する

```bash
pip install -r requirements.txt
./build.sh                     # cmake + build; runs control_test
pytest tests/ -v               # 28 passed (5 files, exercising the C++ modules)
cd python && python run_pipeline.py   # full closed-loop with guardrail (C++ modules)
```

README および `SAFETY_CASE.md` に引用されている数値はすべて、テストスイート（C++ の `*_cpp` モジュールを実行する）と `python/*.py` のエントリポイントによって生成されたものである。
