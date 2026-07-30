# av-stack

[![CI](https://github.com/Toshiaki-Saka/av_stack/actions/workflows/ci.yml/badge.svg)](https://github.com/Toshiaki-Saka/av_stack/actions/workflows/ci.yml)
![Python](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-Apache_2.0-blue)

モジュラー構成の自動運転 (AV) スタックである。量産 AV システムで用いられる古典的な
疎結合パイプライン設計、すなわち センサ → 認識 → 世界モデル → 計画 → 制御 を採り、
プランナが何をしようとも **RSS** の安全距離保証を強制する独立した安全ガードレールを
備える。

すべてが自己完結している。ROS も外部データセットも GPU も不要である。小規模な合成
マルチレーン走行ワールドが、シナリオとクローズドループのテストベッドを生成する。

```
Sensors ──► Perception (fusion) ──► Tracker ──► Planner (IDM) ──► Controller ──► Vehicle
    LiDAR        inv-cov fuse         IMM         lane-change     PID / LQR / MPC   bicycle
    Radar                                         score + IDM
    Camera
                                              ▲                                       │
                                    RSS + TTC Guardrail ◄──────────────────────────────┘
```

本スタックは **C++ ファースト**である。パイプラインのアルゴリズム — センサ、認識、IMM
トラッカ、IDM / レーンチェンジプランナ、RSS ガードレール、占有格子、そして各コントローラ —
はすべて `cpp/include/` 以下の C++17 ヘッダとして実装され、pybind11 を通じて Python に
公開される (`control_cpp`、`world_cpp`、`perception_cpp`、`planning_cpp`、
`guardrail_cpp`、`occupancy_cpp`、`sensors_cpp`、`scenario_cpp`、`localization_cpp`、
`safety_cpp`、`hybrid_cpp`)。Python レイヤ (`python/*.py`) は、それらのモジュールの上に
乗る薄い可視化 / デモ / シナリオ駆動レイヤである。

```
cpp/include/**.hpp  ──pybind11──►  *_cpp modules  ──►  python/*.py (viz, demos, tests)
```

C++ の数値はすべて `control_test` (スタンドアロンの C++ ユニットテスト) と Python テスト
スイートによって検証されており、ガードレールの介入はすべてクローズドループのシナリオで
再現される。

### クイックスタート

**Windows (MSVC) — ワンコマンド。** リポジトリルートの `build_and_run.ps1` が、Python
依存パッケージのインストール、C++ コアのコンフィグとビルド、C++ および Python のテスト
スイートの実行、そしてデモの起動までを一気通貫で行う。

```powershell
.\build_and_run.ps1                          # build + test + full pipeline demo (default)
.\build_and_run.ps1 -SkipBuild               # skip the build, just run the demo
.\build_and_run.ps1 -SkipBuild -SkipTests    # skip build and tests
.\build_and_run.ps1 -Demo acc                # pick a demo (see below)
```

`-Demo` はシナリオを選択する。C++ 側のすべてのシナリオに到達できる。

| `-Demo` | シナリオ | プランナ | 何が見えるか |
|---|---|---|---|
| `pipeline` (デフォルト) | 急ブレーキ、8 m/s² | 両方 | *シナリオではない* — `hard_brake` を 4 通り (正常/故障 × ガードレール on/off) 実行し、比較を出力したうえで、故障 + ガードレールの走行をアニメーション表示する |
| `animation` | 急ブレーキ + カットイン | — | 純 Python のリアルタイムアニメーション。C++ のビルドは不要 |
| `hard_brake` | 急ブレーキ、8 m/s² | 正常 | 最大減速度であってもプランナ単独で車間を維持する。ガードレールは沈黙 |
| `lead_brake` | 中程度のブレーキ、4 m/s² | 正常 | プランナは追従し、その後追い越す。ガードレールは一度も発火しない |
| `mixed` | 低速の先行車 + 左のより速い車 | 正常 | IMM が両エージェントを追跡する。自車は追従状態に落ち着く |
| `acc` | 8 m/s で巡航する先行車 | 正常 | ACC の車間追従。ガードレールは沈黙したままの安全網 |
| `avoidance` | 3 m/s の低速障害物 | 正常 | 左へ `CHANGE_LANE` し、追い越して `CRUISE` に戻る |
| `cut_in` | 隣接車が左から合流してくる | 故障 | プランナは合流を無視する。隣接車が自車線に到達する前に横方向 RSS が発火する |
| `safety_stop` | 急ブレーキ + 左車線が塞がっている | 故障 | 逃げ道なし。RSS ガードレールが停止するまでブレーキをかける |

**任意のプラットフォーム — ビルド不要:**

```bash
pip install -r requirements.txt
python python/animation_demo.py          # real-time animation, no C++ required
```

**さらに読む** — ドキュメントはすべて [`docs_en/`](docs_en/) (英語) と
[`docs_ja/`](docs_ja/) (日本語) にある:
[`ARCHITECTURE.md`](docs_ja/ARCHITECTURE.md) (モジュールごとの詳解) ·
[`SAFETY_CASE.md`](docs_ja/SAFETY_CASE.md) (RSS / ガードレールの安全論証) ·
[`TECHNICAL.md`](docs_ja/TECHNICAL.md) (導出と実装ノート) ·
[`hara.md`](docs_ja/hara.md) · [`requirements_traceability.md`](docs_ja/requirements_traceability.md)。

**English:** [`README.md`](README.md) · [`docs_en/`](docs_en/)

---

## アーキテクチャ

すべてのアルゴリズムは `cpp/include/` 以下の C++ ヘッダ実装であり、`cpp/src/bindings_*.cpp`
の pybind11 モジュールを介して Python に公開される。`python/*.py` のファイル群は
可視化 / デモ / テストのレイヤである。

```
# --- C++ implementations (cpp/include/) ---
control/bicycle.hpp                 # kinematic bicycle: RK4 step + linearise (for LQR/MPC)
control/linalg.hpp                  # minimal dense matrix (no external BLAS dep)
control/lqr.hpp                     # PID (anti-windup) + discrete LQR (DARE backward iter)
control/mpc.hpp                     # linear time-varying MPC (FISTA box-constrained QP)
world/tracking.hpp                  # multi-object IMM tracker (cruise/manoeuvre CV mix)
world/occupancy.hpp                 # probabilistic occupancy grid (Gaussian agent splat)
world/scenario.hpp                  # multi-lane road + scripted agents (brake, cut-in)
sensors/sensors.hpp                 # LiDAR / Radar / Camera object models + covariance R
perception/fusion.hpp               # information-form sensor fusion, Mahalanobis gating
planning/{planner,path,trajectory}.hpp  # IDM planner + lane-change scorer, ref path, traj
safety/guardrail.hpp                # independent RSS + TTC + lateral safety monitor
ad/common/types.hpp                 # shared AD types
ad/localization/{localizer,ekf_localizer}.hpp  # EKF localiser (→ localization_cpp)
ad/safety/speed_governor.hpp        # speed governor / safety envelope
hybrid/{arbiter,confidence}.hpp     # hybrid arbiter + confidence (→ hybrid_cpp)

# --- pybind11 bindings (cpp/src/) ---
bindings_{control,world,scenario,sensors,perception,planning,guardrail,occupancy,
          safety,localization,hybrid}.cpp
                                    # → import control_cpp, world_cpp, scenario_cpp,
                                    #   sensors_cpp, perception_cpp, planning_cpp,
                                    #   guardrail_cpp, occupancy_cpp, safety_cpp,
                                    #   localization_cpp, hybrid_cpp
control_test.cpp                    # standalone C++ unit tests: bicycle, LQR, MPC

# --- Python: visualisation / demo / scenario layer (python/) ---
python/animation_demo.py            # real-time bird's-eye animation + time-series plots
python/run_control_demo.py          # PID vs LQR vs MPC comparison on the double lane-change
python/run_pipeline.py              # full closed-loop pipeline (C++ modules) on hard-brake
python/world.py · path.py · trajectory.py   # support: constants + reference paths for demos
python/{planning,guardrail,perception,sensors,occupancy}.py
                                    # legacy/reference pure-Python implementations; the
                                    #   production code is the *_cpp modules above

# --- tests (pytest tests/, uses the C++ modules) ---
tests/test_control.py · test_modules.py · test_localization.py
tests/test_safety.py · test_hybrid.py

README.md / README.ja.md            # overview (English / Japanese)
docs_en/                            # documentation, English
  ARCHITECTURE.md                   #   in-depth module-by-module walkthrough
  SAFETY_CASE.md                    #   GSN safety-case argument for the RSS guardrail
  TECHNICAL.md                      #   derivations and implementation notes
  hara.md                           #   hazard analysis and risk assessment
  requirements_traceability.md      #   safety goal -> requirement -> component -> test
docs_ja/                            # the same five documents, Japanese
LICENSE                             # Apache-2.0
requirements.txt / build.sh         # dependencies + one-command build & self-check
assets/                             # generated figures
```

このレイアウトは C++ ファーストである。すべてのアルゴリズムは `cpp/include/` に置かれ、
pybind11 を通じて公開される。`python/` はユーザに面した可視化とシナリオのレイヤである。

---

## 車両モデル (`bicycle.hpp`)

キネマティック・バイシクルモデルが、スタック全体を通じた予測およびシミュレーションの
モデルである。

```math
\begin{aligned}
&s = [x, y, \psi, v] \qquad u = [a, \delta] \\
&\dot{x} = v\cos\psi, \quad \dot{y} = v\sin\psi, \quad \dot{\psi} = (v/L)\tan\delta, \quad \dot{v} = a
\end{aligned}
```

積分は (オイラー法ではなく) **RK4** で行う。これは $dt = 0.1$ s における MPC の予測精度に
とって決定的である。RK4 ステップのヤコビアン $A = \partial f/\partial s$、 $B = \partial f/\partial u$ は
中心差分で計算しており、プラントが実際に用いる非線形ステップとの整合性が証明可能な形で
保たれる。検証済み: 線形化残差 $O(\varepsilon^2) = 2.3\times10^{-6}$。

---

## コントローラ (`lqr.hpp`, `mpc.hpp`)

3 つの縦 + 横方向コントローラが、同一のバイシクルプラントと参照経路を共有する。

### PID

縦方向の速度維持 (速度誤差に対する PID、アンチワインドアップ、出力飽和) と、横方向の
経路追従 (横偏差とヘディングに対する P 制御、曲率のフィードフォワード)。単純かつ高速で
あり、トレードオフ比較の基準となる。

### LQR

線形化した 2 状態の横偏差モデル $[e_y, e_\psi]$ に対する離散 LQR である。

$$\dot{e}_y = v\,e_\psi, \quad \dot{e}_\psi = (v/L)\,\delta - v\kappa$$

ゲインは収束するまで ($\lVert \Delta P \rVert < 1\times10^{-10}$) の **DARE 後退反復**によって
計算し、フィードフォワード $\delta_{ff} = \arctan(L\kappa)$ を伴う。検証済み: 8 s 後の横偏差残差 = $8.6\times10^{-14}$ m。

### MPC

15 ステップのホライズンにわたる線形時変 MPC である。各ステージで RK4 ステップを参照軌道
まわりで線形化して $(A_k, B_k, d_k)$ を得たうえで、強凸なボックス制約付き QP へ縮約する。

$$\min\ E^\top \bar{Q} E + \Delta U^\top \bar{R} \Delta U \quad \text{s.t.}\quad u_{min} \le u_{ref} + \Delta U \le u_{max}$$

これを **FISTA** によりゼロから解く (200 反復、ステップ幅 = べき乗反復による 1/L_f)。
アクチュエータ制限 $a \in [-6, 3]$ m/s²、 $\delta \in [-0.6, 0.6]$ rad を明示的に扱う。
検証済み: 目標速度に正確に到達し、制限は 1 % 以内で遵守される。

---

## センサと認識 (`cpp/include/sensors/sensors.hpp`, `cpp/include/perception/fusion.hpp`)

3 つのセンサモデルがあり、それぞれ観測共分散 $R$ を伴う検出を返す。

| センサ | ノイズモデル | 固有の出力 |
|---|---|---|
| **LiDAR** | 等方性 $\sigma = 0.15$ m | 位置のみ |
| **Radar** | 異方性: $\sigma_r = 0.4$ m、 $\sigma_{lat} = 1.2$ m (回転済み) | ドップラーによる視線方向速度 |
| **Camera** | $\sigma_{bearing} = 0.6$°、距離の相対誤差 10 % (回転済み) | 物体クラスラベル |

フュージョン (**`perception/fusion.hpp`**) は、センサ横断の検出をマハラノビス距離ゲーティング
($\chi^2 < 9.21$、2 自由度、99 % ゲート) によりグループ化し、各クラスタを情報形式で統合する。

$$R_{fused} = \left(\sum_i R_i^{-1}\right)^{-1}, \quad z_{fused} = R_{fused} \sum_i R_i^{-1} z_i$$

これは独立なガウス観測の最尤結合である。レーダのドップラーは、新規トラックの速度推定に
初期値を与える。

---

## 多物体トラッカ (`cpp/include/world/tracking.hpp`)

物体ごとの **IMM (Interacting Multiple Model)** トラッカ (`struct IMM`) であり、プロセス
ノイズの異なる 2 つの等速度カルマンフィルタ — 低ノイズの*巡航*モデルと高ノイズの*機動*
モデル — を混合する。これにより、巡航中は推定が引き締まったままでありながら、エージェントが
ブレーキやカットインをした際には素早く反応する。状態は $[x, y, v_x, v_y]$ であり、モード確率は
各モデルの観測尤度から更新され、統合推定値は確率で重み付けした混合となる。更新を駆動する
のは (共分散 $R$ を伴う) 融合後の位置検出である。データアソシエーションはマハラノビス
ゲート付きの最近傍法で行い、トラックは `min_hits` 回の一貫した更新の後に*確定 (confirmed)*
となる。検証済み: 3 s のコールドスタート後、平均位置誤差 < 0.6 m、速度誤差 < 0.4 m/s。

---

## プランナ (`cpp/include/planning/planner.hpp`)

候補となる各車線 (維持 / 左へ変更 / 右へ変更) について、プランナは次を行う。

1. **IDM ロールアウト** — **Intelligent Driver Model** を用いて、予測された同一車線内の
   先行車に対し、自車を縦方向にロールアウトする。

```math
\begin{aligned}
   s^* &= s_0 + vT + \frac{v\cdot\Delta v}{2\sqrt{a\cdot b}} \\
   a_{IDM} &= a\left[1 - (v/v_{des})^4 - (s^*/gap)^2\right] \quad \text{clipped to } [-1.5b, a]
   \end{aligned}
```

2. **スムーズステップ横方向遷移** — 現在の車線から目標車線へ $t_{change} = 3$ s かけてブレンドする。

3. **コスト** — 速度維持、進行、横方向の快適性 (最大 $v^2 \kappa$)、車線選好、予測されたエージェントとの最小クリアランスでスコア付けする。いずれかのエージェントの安全半径に侵入する軌道は**棄却**される。

すべての候補が安全でない場合は `EMERGENCY_SLOW` (1.5b で減速) にフォールバックし、判断をガードレールに委ねる。検証済み: IDM の車間が維持されること、隣接スロットが空いているときにのみレーンチェンジが受理されること。

---

## ガードレール (`cpp/include/safety/guardrail.hpp`)

プランナを信用しない、**独立した doer-checker** である。毎タイムステップで 3 つのチェックを
行う。

### 1. RSS 縦方向 (Shalev-Shwartz et al., 2017)

最小安全車間距離。すなわち、先行車が $b_{max}$ でブレーキする一方、自車が $\rho$ 秒 (反応時間)
のあいだ加速し、その後 $b_{min}$ でブレーキしたとしても安全であるような車間である。

$$d_{RSS} = v_{ego}\,\rho + \tfrac{1}{2} a \rho^2 + \frac{(v_{ego} + \rho a)^2}{2b} - \frac{v_{lead}^2}{2b_{lead}}$$

### 2. 衝突余裕時間 (Time-to-collision)

$TTC = gap / (v_{ego} - v_{lead})$ がしきい値 (2.5 s) を下回ること。

### 3. 横方向 RSS

危険なカットインを検出する。RSS に従えば、状況が危険となるのは横方向*かつ*縦方向の安全
距離が同時に侵害されたときのみであり、したがって両者がともに成立しなければならない。

$$d_{lat}(v_{lat}) = \mu + \ell(0) + \ell(|v_{lat}|), \qquad \ell(v) = v\rho + \tfrac{1}{2}a_{lat}\rho^2 + \frac{(v + \rho a_{lat})^2}{2b_{lat}}$$

発火するのは $|\Delta y| < d_{lat}$ *かつ* $-L_{veh} \le \Delta x \le d_{RSS} + L_{veh}$ のときである。
横方向距離は車線幅の半分を超えて届く ($v_{lat} = 2$ m/s において $3.82$ m、対する
$\text{LANE}/2 = 1.75$ m)。これこそが、隣接車がまだ隣の車線にいるうちにこのチェックを発火
させるものであり、隣接車が同一車線の先行車となってしまい結局は縦方向チェックが捉えたで
あろう状態になってから発火するのではない。

さらに 2 つの条件がこれをゲートする。トラックの横方向速度が信頼できること
($|v_y| \le 3$ m/s、センサのゴーストを棄却する)、および実際に自車へ接近していること
($\Delta y \cdot v_y < 0$、したがって遠ざかっていく隣接車のためにブレーキはしない) である。
これらは安全上のクレジットを持たない。可用性を守るために存在する。

いずれかのチェックが発火すると、ガードレールは `hold` ステップのあいだ**ラッチ**し、
プランナの指令を緊急ブレーキ ($-6$ m/s²) で置き換える。

ガードレールの限界についての率直な所見: RSS チェックは最悪ケースのパラメータ
(先行車について $b_{max} = 8$ m/s²) を用いる。これらのパラメータのキャリブレーション誤差は、
そのまま車間の誤差に直結する。セーフティケースはこの点を明示している。

---

## 占有格子 (`cpp/include/world/occupancy.hpp`)

予測された各エージェントの軌道を、前方の道路グリッド上に車両フットプリントのガウス分布
としてスプラットし、ステップごとに不確かさを $\sigma(k) = \sigma_0 + growth\cdot k$ で膨張させる。
エージェント間は確率的 OR で結合する。プランナには**ソフトコスト**を、ガードレールには
**ハード拒否 (hard veto)** のしきい値を提供する — 離散的な衝突判定に対する連続的な代替手段
である。検証済み: 10 m² の占有セルは、真のエージェント位置において > 0.9 の確率を示す。

---

## ガードレールに関する率直な結果 (そしてそれが重要な理由)

`scenario_hard_brake` (先行車が 8 m/s² — パラメータ化された最大値 — で減速する) でスタックを
テストすると、IDM プランナは反応するものの、快適減速度を前提としているために、フル
ブレーキが予測される前に車間を詰める計画を立ててしまう。ガードレールは最悪ケースの RSS
仮定のもとで安全性を再導出し、間に合うようにブレーキをかける。ガードレールを無効にした
場合と有効にした場合の比較は次の通りである。

| | 追突 | 最小車間 | 結果 |
|---|---|---|---|
| プランナのみ (ガードレールなし) | **あり** | **−0.8 m** | 衝突 |
| ガードレールあり | **なし** | **+3.1 m** | 安全に停止 |

カットインシナリオでは、隣接車がまだ隣の車線にいるうちに横方向 RSS チェックが発火する —
横方向の安全距離は 2 m/s の合流において 3.82 m に達し、車線幅の半分を大きく超える。縦方向
チェック単独と比べると 0.5 s 早く反応し (1.2 s 対 1.7 s)、1.9 m 多くのクリアランスを保つ。

ガードレールは、本スタックの中で最も単純で最も監査しやすいコンポーネントである。これは
意図的なものである。機能安全のためには、安全モニタは*独立に実装・検証可能*でなければ
ならず、それ自体がモニタを必要とするような複雑なモジュールであってはならない。

---

## ビルド

C++17 コンパイラ、CMake $\ge 3.18$、および pybind11 を備えた Python $\ge 3.9$ を必要とする。

```bash
pip install pybind11 numpy matplotlib pytest
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

あるいは付属のスクリプトを使う (Linux / macOS)。

```bash
./build.sh
```

Windows (MSVC) では、Visual Studio の開発者プロンプトから実行する。

```bat
cmake -S . -B build
cmake --build build --config Release
```

検証:

```bash
./build/control_test              # Linux / macOS
build\Release\control_test.exe   # Windows
```

期待される出力:

```
CONTROL TEST PASS
```

---

## 実行

アニメーションデモ (純 Python、ビルド不要):

```bash
python python/animation_demo.py
```

コントローラの比較 (ダブルレーンチェンジにおける PID 対 LQR 対 MPC):

```bash
cd python && python run_control_demo.py
```

フルクローズドループ・パイプライン (センサ → 認識 → トラッキング → 計画 → ガードレール → MPC):

```bash
cd python && python run_pipeline.py
```

テストスイート:

```bash
pytest tests/ -v
```

---

## 検証済みの結果

| 検証項目 | 結果 |
|---|---|
| バイシクル RK4 の線形化残差 | $O(\varepsilon^2) = 2.3\times10^{-6}$ (非線形ステップと整合) |
| 8 s 後の LQR 横偏差残差 | $8.6\times10^{-14}$ m (実質ゼロ) |
| MPC の速度追従 | $v = 12.00$ m/s (目標 12)、 $\max\lvert a\rvert = 3.00$ (制限 3.0) |
| MPC の横方向復帰 | $y = 0.000$ m (目標 0)、 $\max\lvert\delta\rvert = 0.600$ (制限 0.6) |
| トラッカの位置誤差 (定常状態) | < 0.6 m (平均)、< 0.4 m/s (速度) |
| ガードレール — 急ブレーキシナリオ | 追突 0 件 (ガードレールなしでは衝突) |
| ガードレール — カットインシナリオ | 横方向 RSS は縦方向チェック単独より 0.5 s 早く反応し (1.2 s 対 1.7 s)、クリアランス +1.9 m |
| Python テストスイート | 33 件パス (`pytest tests/`、5 ファイル、C++ モジュールを実行) |

---

## 拡張する

- **より長いホライズン、あるいは非線形 MPC。** `MPC::solve` は自己完結した FISTA ループである。線形化を逐次 QP ステップに置き換えれば SQP になる。
- **実センサ。** `SensorSuite` を ROS トピックのサブスクライバに差し替えればよい。フュージョンとトラッキングのインタフェースはセンサ非依存である。
- **シナリオの追加。** `cpp/include/world/scenario.hpp` に `brake` または `cut_in` プロファイルを持つエージェントを追加する。プランナとガードレールはシナリオ非依存である。
- **確率的プランニング。** `OccupancyGrid` は既にソフトコストを生成している。これを重み付き項として `Planner._cost` に組み込めばよい。

---

## References

- Shalev-Shwartz, Shammah & Shashua, *On a Formal Model of Safe and Scalable Self-Driving Cars* (RSS), 2017.
- Treiber, Hennecke & Helbing, *Congested Traffic States in Empirical Observations and Microscopic Simulations* (IDM), 2000.
- Mayne, Rawlings, Rao & Scokaert, *Constrained Model Predictive Control: Stability and Optimality*, Automatica 2000.
- Beck & Teboulle, *A Fast Iterative Shrinkage-Thresholding Algorithm for Linear Inverse Problems* (FISTA), SIAM J. Imaging Sci. 2009.
- Werling, Ziegler, Kammel & Thrun, *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*, ICRA 2010.
