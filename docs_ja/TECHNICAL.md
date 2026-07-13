# テクニカルリファレンス: av_stack — モジュラー型自動運転スタック

> 英語版: [`../docs_en/TECHNICAL.md`](../docs_en/TECHNICAL.md)

> **想定読者:** 本ドキュメントは、状態空間制御理論・ベイズフィルタリング・凸最適化に関する
> 基礎知識を前提とする。目的は、コードの動作を説明することではなく、スタック内のすべての
> アルゴリズムを再現・拡張・監査するための数学的基盤を読者に与えることにある。

---

## 目次

1. [システムアーキテクチャ](#1-システムアーキテクチャ)
2. [車両運動力学 — キネマティック自転車モデル](#2-車両運動力学--キネマティック自転車モデル)
3. [線形代数バックエンド](#3-線形代数バックエンド)
4. [制御レイヤ](#4-制御レイヤ)
   - 4.1 [アンチワインドアップ付き PID](#41-アンチワインドアップ付き-pid)
   - 4.2 [DARE による離散時間 LQR](#42-dare-による離散時間-lqr)
   - 4.3 [FISTA を用いた線形時変 MPC](#43-fista-を用いた線形時変-mpc)
5. [センサモデル](#5-センサモデル)
6. [知覚 — 情報形式センサフュージョン](#6-知覚--情報形式センサフュージョン)
7. [ワールドモデル — IMM カルマントラッカ](#7-ワールドモデル--imm-カルマントラッカ)
8. [モーションプランニング](#8-モーションプランニング)
9. [安全ガードレール — RSS モニタ](#9-安全ガードレール--rss-モニタ)
10. [確率的占有予測](#10-確率的占有予測)
11. [検証と数値結果](#11-検証と数値結果)
12. [参考文献](#12-参考文献)

---

## 1. システムアーキテクチャ

本スタックは、複雑さが増し、安全クリティカル性が下がる順に並んだ 4 つの同心レイヤとして
構成されている。

```
Sensors (LiDAR / Radar / Camera)
       │
       ▼
┌──────────────────────────────────────────────────────────────────┐
│  Perception  ·  sensors.py → perception.py                       │
│  Information-form fusion over anisotropic per-sensor covariances │
└─────────────────────────────┬────────────────────────────────────┘
                              │ FusedDetection (z, R, class, v_prior)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  World Model  ·  tracking.hpp (C++, pybind11)                    │
│  IMM Kalman filter per track; greedy Mahalanobis association      │
│  Constant-velocity prediction → predicted trajectories            │
└─────────────────────────────┬────────────────────────────────────┘
                              │ confirmed tracks + predictions
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Planner  ·  planning.py  +  occupancy.py                        │
│  IDM longitudinal · smoothstep lateral · multi-lane cost search  │
│  Probabilistic occupancy grid for soft path cost                 │
└─────────────────────────────┬────────────────────────────────────┘
                              │ Trajectory (x,y,v,κ)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Safety Guardrail  ·  guardrail.py  (independent monitor)        │
│  RSS longitudinal · TTC · RSS lateral                            │
└─────────────────────────────┬────────────────────────────────────┘
                              │ (a, δ) or (−b_emergency, δ_hold)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Controller  ·  mpc.hpp / lqr.hpp (C++, pybind11)               │
│  LTV-MPC or Lateral LQR + longitudinal PID                       │
└─────────────────────────────┬────────────────────────────────────┘
                              │ actuator commands → bicycle plant
                              ▼
                         Vehicle (bicycle.hpp, RK4)
```

**関心の分離。** 安全ガードレールは意図的にプランナの*後段*、コントローラの*前段*に
配置されている。ガードレールはプランナではなくワールドモデルから直接データを読むため、
プランニングの故障がガードレールを侵害することはない。これは IEC 61508 / ISO 26262 が
SIL 2 以上のモニタに要求する doer–checker パターンに対応している。

---

## 2. 車両運動力学 — キネマティック自転車モデル

### 2.1 連続時間モデル

キネマティック自転車モデルは、四輪車両を 1 本の後車軸と前車軸の 1 つの操舵角に縮約する。
状態 $s = [x, y, \psi, v]^\top \in \mathbb{R}^4$、入力 $u = [a, \delta]^\top \in \mathbb{R}^2$ とすると:

$$\begin{aligned}
\dot{x} &= v \cos \psi \\
\dot{y} &= v \sin \psi \\
\dot{\psi} &= (v / L) \tan \delta \\
\dot{v} &= a
\end{aligned}$$

ここで $L = 2.7$ m はホイールベースである。本モデルは低〜中程度のスリップ角領域で妥当であり、
タイヤ限界に達しない高速道路速度域では、キネマティック仮定は動力学 (力ベース) モデルに対して
数パーセント以内の誤差で成立する。

### 2.2 数値積分 — 4 次 Runge–Kutta 法

離散ステップ $s_{k+1} = \Phi(s_k, u_k, dt)$ は、古典的な 4 次 Runge–Kutta 法で計算する。

$$\begin{aligned}
k_1 &= f(s_k, u_k) \\
k_2 &= f(s_k + (dt/2) k_1,\ u_k) \\
k_3 &= f(s_k + (dt/2) k_2,\ u_k) \\
k_4 &= f(s_k + dt\, k_3,\ u_k) \\
s_{k+1} &= s_k + \tfrac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)
\end{aligned}$$

**なぜ Euler 法ではなく RK4 か。** Euler 法の局所打ち切り誤差は $O(dt^2)$ であり、
$dt = 0.1$ s、$v = 13$ m/s ではヨーレート項 $v/L \cdot \tan \delta$ が毎秒およそ
0.5 m/s のヨーレート誤差を蓄積してしまう。RK4 は局所的に $O(dt^5)$ であり、これを
サブミリメートル領域まで低減する。MPC の予測ホライズンが 15 ステップ (1.5 s) にわたるため、
予測モデルが不正確だと最適入力に系統的なバイアスが乗るので、この点は重要である。

### 2.3 中心差分による離散時間ヤコビアン

MPC と LQR の線形化のため、離散時間ヤコビアンが必要となる。

$$\begin{aligned}
A_k &= \partial \Phi / \partial s \,\big|_{(s_k, u_k)} \in \mathbb{R}^{4\times 4} \\
B_k &= \partial \Phi / \partial u \,\big|_{(s_k, u_k)} \in \mathbb{R}^{4\times 2}
\end{aligned}$$

これらは中心差分により数値的に計算する。

$$\begin{aligned}
A_k(i,j) &= \big[ \Phi(s_k + \varepsilon e_j, u_k) - \Phi(s_k - \varepsilon e_j, u_k) \big]_i / (2\varepsilon) \\
B_k(i,j) &= \big[ \Phi(s_k, u_k + \varepsilon e_j) - \Phi(s_k, u_k - \varepsilon e_j) \big]_i / (2\varepsilon)
\end{aligned}$$

ここで $\varepsilon = 10^{-6}$ である。解析的ヤコビアンに対する最大の利点は**整合性**にある。
差分ヤコビアンは、プラントが使用するのと同一の RK4 ステップと必ず整合するため、LTV 誤差モデル
$e_{k+1} \approx A_k e_k + B_k \Delta u_k + d_k$ にモデル化ミスマッチが生じない。解析的
ヤコビアンでは RK4 の各ステージを通した連鎖律の導出が必要となり、符号ミスを招きやすいうえ、
積分手法を変更するたびに無効化されてしまう。

**検証。** 典型的な動作点 $(s_0, u_0)$ において、線形化残差は次を満たす。

$$\lVert \Phi(s_0 + \delta s, u_0) - (\Phi(s_0, u_0) + A_k \delta s) \rVert / \lVert \delta s \rVert \approx 2.3 \times 10^{-6} \quad \text{(second-order)}$$

これは、ヤコビアンが中心差分に期待される $O(\varepsilon^2)$ の精度で正しいことを確認するものである。

---

## 3. 線形代数バックエンド

すべての数値計算は、外部依存を持たない最小限のヘッダオンリー密行列ライブラリ (`linalg.hpp`) を
用いる (Eigen、BLAS、LAPACK のいずれも使用しない)。設計上のトレードオフは、$O(n^3)$ の密行列
アルゴリズムと引き換えに、移植性とセットアップコストゼロを得ることである。これが許容できるのは、
すべての行列が高々 ~$30 \times 30$ 程度 ($N = 15$、$m = 2$ の condensed MPC QP で
$N \cdot m = 30$) だからである。

**`solve(A, b)` — 部分ピボット選択付き Gauss–Jordan 法。** 各列 $c$ について:

1. ピボット行 $p = \operatorname*{argmax}_{i \ge c} |A(i,c)|$ を求める。
2. $A$ と $b$ の行 $c$ と行 $p$ を入れ替える。
3. 行 $c$ を対角要素で除算する。
4. 他のすべての行から列 $c$ を消去する。

ピボットが $10^{-12}$ を下回る場合は、微小な正則化項 $A(c,c) \mathrel{+}= 10^{-9}$ を加えて
特異に近いケースを穏当に処理する (これは DARE の反復ごく初期、$P$ がまだ $Q$ に近い段階で発生する)。

**`inv(A)` = `solve(A, I)`。** トラッカのイノベーション共分散の逆行列計算 ($2\times2$ 行列) に
のみ使用するため、コストは無視できる。

---

## 4. 制御レイヤ

### 4.1 アンチワインドアップ付き PID

積分項クランプを備えた標準的な三項制御器である。

$$u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d\, \dot{e}(t)$$

積分項は Euler 積分、微分項は後退差分で離散化する。飽和は 2 段階を独立に適用する。

- **積分クランプ** `[i_min, i_max]`: 累積した積分値が総和に寄与する前に制限し、アクチュエータが
  飽和したときのワインドアップを防止する。
- **出力クランプ** `[out_min, out_max]`: 最終指令値を制限する。

これは古典的な*条件付きアンチワインドアップ*方式である (Åström & Wittenmark §3.5)。積分項は
総和の後ではなく前でクランプされるため、ワインドアップは事後補正されるのではなく未然に防止される。

### 4.2 DARE による離散時間 LQR

**問題設定。** 離散時不変線形システム $e_{k+1} = A e_k + B u_k$ に対し、無限ホライズン二次コストを
最小化する状態フィードバックゲイン $K$ を求める。

$$J = \sum_{k=0}^{\infty} (e_k^\top Q e_k + u_k^\top R u_k), \quad u_k = -K e_k$$

**DARE による解。** 最適ゲインは次を満たす。

$$\begin{aligned}
P &= Q + A^\top P A - A^\top P B (R + B^\top P B)^{-1} B^\top P A \quad \text{(Discrete Algebraic Riccati Equation)} \\
K &= (R + B^\top P B)^{-1} B^\top P A
\end{aligned}$$

実装では **価値反復** (有限ホライズン近似における後退再帰であり、無限ホライズン解へ収束する) に
より DARE を解く。

$$\begin{aligned}
P_0 &= Q \\
P_{n+1} &= Q + A^\top P_n A - A^\top P_n B (R + B^\top P_n B)^{-1} B^\top P_n A
\end{aligned}$$

これは不動点近傍で安定化解 $P^*$ に二次収束する。収束判定は
$\lVert P_{n+1} - P_n \rVert_1 < 10^{-10}$ (2 状態モデルでは通常 30 反復未満) であり、
`iters = 1000` は保守的な上限値である。

**横方向経路追従モデル。** LQR は 2 状態の横方向誤差モデル $e = [e_y, e_\psi]^\top$ に適用する。

$$\begin{aligned}
e_y' &= v \cdot e_\psi \\
e_\psi' &= (v/L)\, \delta - v \kappa_{ref}
\end{aligned}$$

$dt$ で Euler 離散化すると:

$$A = \begin{bmatrix} 1 & v \cdot dt \\ 0 & 1 \end{bmatrix}, \quad
B = \begin{bmatrix} 0 \\ (v/L) \cdot dt \end{bmatrix}, \quad
Q = \operatorname{diag}(q_{e_y}, q_{e_\psi}), \quad
R = [r_\delta]$$

フィードフォワード項 $\delta_{ff} = \arctan(L \kappa_{ref})$ が曲率項を打ち消すため、LQR は残差
誤差の抑制のみを担えばよい。ゲイン $K$ は各ステップで現在速度 $v$ (停車時の特異性を避けるため
0.5 m/s でクランプ) を用いて再計算されるので、これは固定ゲインではなく**オンライン・ゲイン
スケジューリング LQR** である。

**定常性能。** 12 m/s のダブルレーンチェンジ軌道において、横方向 LQR は定常横偏差
**$8.6 \times 10^{-14}$ m** (機械精度) を達成する。これは、曲率フィードフォワードと LQR
フィードバックが協調して、この定速軌道に対し参照を厳密に追従していることを裏付けている。

### 4.3 FISTA を用いた線形時変 MPC

#### 4.3.1 問題定式化

参照軌道 $\{(s_{ref,k}, u_{ref,k})\}_{k=0}^{N}$ を、後退ホライズン $N = 15$ ステップ
($dt = 0.1$ s、1.5 s 先読み) にわたり、ボックス制約のもとで追従する。

$$\begin{aligned}
\min \quad & \sum_{k=1}^{N} e_k^\top \bar{Q} e_k + \sum_{k=0}^{N-1} \Delta u_k^\top \bar{R} \Delta u_k \\
\text{s.t.} \quad & e_{k+1} = A_k e_k + B_k \Delta u_k + d_k, \quad e_k = s_k - s_{ref,k} \\
& a_{min} \le u_{ref,k}(0) + \Delta u_k(0) \le a_{max} \\
& \delta_{min} \le u_{ref,k}(1) + \Delta u_k(1) \le \delta_{max}
\end{aligned}$$

ここで $\bar{Q} = \operatorname{diag}(q_x, q_y, q_\psi, q_v)$ と
$\bar{R} = \operatorname{diag}(r_a, r_\delta)$ はホライズン方向にブロック対角として繰り返され、
$d_k = \Phi(s_{ref,k}, u_{ref,k}) - s_{ref,k+1}$ は非線形参照軌道からのアフィン defect (欠損項)
である。

#### 4.3.2 ステージごとの線形化

ステージ $k$ において、RK4 ステップを $(s_{ref,k}, u_{ref,k})$ のまわりで中心差分 (2.3 節) により
線形化し、$(A_k, B_k)$ を得る。これにより**線形時変 (LTV)** 誤差ダイナミクスが得られる。

$$e_{k+1} = A_k e_k + B_k \Delta u_k + d_k$$

defect $d_k$ は Taylor 展開の剰余項を吸収し、すべての $k$ について $e_k = 0$ が
$s_k = s_{ref,k}$ と整合することを保証する。これは制約が正しく機能するために必要な性質である。

#### 4.3.3 Condensing (凝縮)

積み上げ状態ベクトル $E = [e_1^\top, \dots, e_N^\top]^\top \in \mathbb{R}^{Nn}$ と入力摂動
$\Delta U = [\Delta u_0^\top, \dots, \Delta u_{N-1}^\top]^\top \in \mathbb{R}^{Nm}$ を定義する。
前向き再帰により:

$$E = S_x e_0 + S_u \Delta U + \text{offset}$$

**凝縮感度行列** $S_u \in \mathbb{R}^{Nn \times Nm}$ は下三角ブロック構造をもつ。

$$S_u[(k)n+i,\, (j)m+c] = \big[ A_{k-1} \cdots A_{j+1} B_j \big](i,c) \quad \text{for } j \le k-1$$

また offset $\text{offset}[k \cdot n : (k+1) \cdot n]$ は、$e_0$ とすべての defect
$d_0, \dots, d_{k-1}$ を $A$ 行列の積を通して伝播させる。

**凝縮 QP** (E を消去):

$$\begin{aligned}
& \min_{\Delta U} \ \tfrac{1}{2} \Delta U^\top H \Delta U + g^\top \Delta U \\
& \text{s.t.} \quad lo \le \Delta U \le hi \\[4pt]
& H = 2(S_u^\top \bar{Q}_{blk} S_u + \bar{R}_{blk}) \quad \text{(positive definite)} \\
& g = 2 S_u^\top \bar{Q}_{blk}\, \text{offset} \\
& lo[k \cdot m + j] = u_{min}[j] - u_{ref,k}[j] \\
& hi[k \cdot m + j] = u_{max}[j] - u_{ref,k}[j]
\end{aligned}$$

$\bar{R}_{blk}$ の対角が厳密に正であるため、$H$ は常に正定である。

#### 4.3.4 ソルバ: ボックス射影付き FISTA

**なぜ FISTA か。** 凝縮 QP の制約集合はボックスであり、その射影 $\Pi_{[lo,hi]}$ は成分ごとの
クランプ、すなわち閉形式の $O(Nm)$ 演算である。強凸な目的関数と閉形式射影の組み合わせに対し、
FISTA (Fast Iterative Shrinkage-Thresholding Algorithm; Beck & Teboulle, 2009) は目的関数
ギャップについて $O(1/k^2)$ の収束を達成する (素朴な射影勾配法は $O(1/k)$)。内点法はより速い
漸近収束を得られるが、LP/QP ソルバへの依存を要し、ここでの問題規模 ($Nm = 30$) には過剰である。

**アルゴリズム** (モーメンタムのリスタートは未実装。素の FISTA で十分):

$$\begin{aligned}
& \text{Initialise:} \ x_0 = y_0 = 0 \in \mathbb{R}^{Nm}, \ t_0 = 1 \\
& \text{For } i = 0, 1, \dots, I-1: \\
& \quad grad = 2 H y_i + g \quad \text{(gradient at } y_i\text{)} \\
& \quad x_{i+1} = \Pi_{[lo,hi]}( y_i - \alpha \cdot grad ) \quad \text{(proximal step + projection)} \\
& \quad t_{i+1} = (1 + \sqrt{1 + 4 t_i^2}) / 2 \\
& \quad y_{i+1} = x_{i+1} + \frac{t_i - 1}{t_{i+1}}(x_{i+1} - x_i) \quad \text{(Nesterov momentum)}
\end{aligned}$$

**ステップサイズ。** ステップサイズは $\alpha = 1/L_f$ とする。ここで
$L_f = \lambda_{\max}(2H)$ は勾配 $2Hy + g$ の Lipschitz 定数である。最大固有値は**べき乗法**を
60 回反復して推定する。

$$\begin{aligned}
v_0 &= (1/\sqrt{Nm})\, \mathbf{1}_{Nm} \\
w_{k+1} &= H v_k / \lVert H v_k \rVert, \quad \lambda \approx \lVert H v_k \rVert
\end{aligned}$$

べき乗反復は $(\lambda_1/\lambda_2)^k$ の速度で収束する。$H$ が良条件 ($\bar{R}$ が正則化として
働くため通常はそうなる) の場合、60 反復は保守的な設定である。べき乗法の推定値に $10^{-9}$ を
加えることで、ステップサイズが有限であることを保証している。

**収束性。** $I = 200$ 反復に対し、最適性ギャップは次を満たす。

$$f(x_i) - f(x^*) \le L_f \lVert x_0 - x^* \rVert^2 / (2i^2)$$

(Beck & Teboulle 定理 4.4)。実際には 200 反復は必要量をはるかに上回っており、ここで用いる
ホライズンと重みでは、約 40 反復以降、反復解の変化量は $10^{-8}$ を下回る。

---

## 5. センサモデル

3 つのセンサはいずれも**物体検出レベル** (生信号レベルではない) でモデル化されている。各センサは
可視性 (距離 $r \le r_{max}$、方位 $|\beta| \le \theta_{fov}/2$) を判定し、未検出確率 $p_{miss}$ を
適用したうえで、観測値 $z \in \mathbb{R}^2$ (ワールド座標系の位置) とノイズ共分散
$R \in \mathbb{R}^{2\times 2}$ をもつ検出を返す。

### 5.1 LiDAR

LiDAR の空間分解能はほぼ等方的であるため、ノイズも等方としてモデル化する。

$$\begin{aligned}
z &= [x, y]^\top + n, \quad n \sim \mathcal{N}(0, \sigma_L^2 I_2), \quad \sigma_L = 0.15 \text{ m} \\
R_L &= \sigma_L^2 I_2
\end{aligned}$$

パラメータ: $r_{max} = 80$ m、$\theta_{fov} = 120°$、$p_{miss} = 0.05$。

### 5.2 Radar

Radar は距離分解能が高い一方、クロスレンジ (方位) 分解能は低い。これは FMCW (周波数変調連続波)
方式に典型的なトレードオフである。ノイズは視線方向 (LOS) に沿って異方的となる。

$$\begin{aligned}
\text{Noise in LOS frame:} \quad & n_{LOS} \sim \mathcal{N}(0, \operatorname{diag}(\sigma_r^2, \sigma_{lat}^2)) \\
\text{Noise in world frame:} \quad & n = \operatorname{Rot}(\alpha)\, n_{LOS}, \quad \alpha = \psi_{ego} + \beta_{detection} \\
R_{Radar} &= \operatorname{Rot}(\alpha) \cdot \operatorname{diag}(\sigma_r^2, \sigma_{lat}^2) \cdot \operatorname{Rot}(\alpha)^\top
\end{aligned}$$

ここで $\sigma_r = 0.4$ m (距離方向)、$\sigma_{lat} = 1.2$ m (横方向) である。さらに Radar は
**ドップラー視線速度** $v_r = (v_x \cos \alpha + v_y \sin \alpha) + n_v$ を計測する。ここで
$n_v \sim \mathcal{N}(0, \sigma_{vr}^2)$、$\sigma_{vr} = 0.1$ m/s である。パラメータ:
$r_{max} = 120$ m、$\theta_{fov} = 90°$、$p_{miss} = 0.05$。

### 5.3 Camera

Camera は角度 (方位) 分解能が高い一方、距離分解能は低く、しかも距離とともに悪化する (見かけの
大きさに基づく距離推定のため)。

$$\begin{aligned}
\text{Bearing noise:} \quad & \sigma_b = 0.6° \quad \text{(}\sigma_{bearing}\text{ in radians)} \\
\text{Range relative error:} \quad & \sigma_{range} / r = 10\% \\
\text{Noise in polar frame:} \quad & (n_r, n_b) \sim \mathcal{N}(0, \operatorname{diag}((0.10 r)^2, r^2 \sigma_b^2)) \\
\text{Noise in world frame:} \quad & n = \operatorname{Rot}(\alpha) \cdot [n_r, n_b]^\top \\
R_{Camera} &= \operatorname{Rot}(\alpha) \cdot \operatorname{diag}((0.10 r)^2, (r \sigma_b)^2) \cdot \operatorname{Rot}(\alpha)^\top
\end{aligned}$$

パラメータ: $r_{max} = 70$ m、$\theta_{fov} = 70°$、$p_{miss} = 0.04$。Camera は物体のクラス
ラベル $kind \in \{car, truck, \dots\}$ も提供する。

---

## 6. 知覚 — 情報形式センサフュージョン

### 6.1 センサ間クラスタリング

3 つのセンサすべての検出は、まず同一物理物体を指す観測どうしを対応づけるためにクラスタリング
される。**マハラノビス・ゲーティング**を伴う貪欲最近傍法を用いる。

$$\begin{aligned}
d^2(i, j) &= \Delta z_{ij}^\top (R_i + R_j)^{-1} \Delta z_{ij} < \gamma_{gate} = 9.21 \\
\Delta z_{ij} &= z_i - z_j
\end{aligned}$$

ゲート $\gamma_{gate} = 9.21$ は $\chi^2(2)$ 分布 (自由度 2、すなわち $z$ の次元) の 99 パーセンタイル
である。固定的なユークリッド距離ではなく**和共分散** $R_i + R_j$ を用いることは、Camera 検出に
とって決定的に重要である。50 m 地点の Camera は $\sigma_{range} \approx 5$ m をもつため、素朴な
ユークリッド・ゲーティングでは正しい対応づけを棄却してしまう。和共分散ゲートは Camera の不確かさを
正しく扱い、距離方向に数メートルの食い違いがあっても LiDAR 検出と対応づけることができる。

### 6.2 情報形式フュージョン

各クラスタ内では、検出値を**情報形式の重み付き平均**により融合する。これは、独立なガウスノイズを
仮定したときの最尤推定値である。

$$\begin{aligned}
R_{fused}^{-1} &= \sum_i R_i^{-1} \quad \text{(sum of Fisher informations)} \\
z_{fused} &= R_{fused} \cdot \sum_i R_i^{-1} z_i \quad \text{(information-weighted mean)}
\end{aligned}$$

これは、独立な複数の観測を逐次的に適用したカルマン更新と等価である (順序不変)。性質は次のとおり。
- 融合後の共分散は、常に個々の $R_i$ のいずれよりも小さい。
- 不確かさが大きいセンサ ($R_i^{-1}$ が小さいセンサ) ほど、融合平均への寄与が小さい。
- すべての観測が同一の状態空間 $\mathbb{R}^2$ にあるため、線形化は不要である。

### 6.3 ドップラーによる速度事前情報

クラスタに Radar 検出が含まれる場合、そのドップラー値 `v_r` は新規カルマントラックを初期化する
ための粗い速度事前値を与える。

$$\begin{aligned}
\alpha_{LOS} &= \operatorname{arctan2}(z_{fused}[1], z_{fused}[0]) \quad \text{(LOS angle from origin, approximate)} \\
v_{prior} &= v_r \cdot [\cos \alpha_{LOS}, \sin \alpha_{LOS}]^\top
\end{aligned}$$

これによりカルマンフィルタの速度状態がウォームスタートされ、コールドスタート時の速度誤差が
$O(v_{true})$ から $O(\sigma_{vr} \cdot \text{angular\_error})$ へ低減される。

---

## 7. ワールドモデル — IMM カルマントラッカ

### 7.1 等速度カルマンフィルタ (CV-KF)

各トラックの単一モデルフィルタは、等速度予測モデルを用いる。

$$\begin{aligned}
\text{State:} \quad & x = [x_p, y_p, v_x, v_y]^\top \in \mathbb{R}^4 \\
\text{Measurement:} \quad & z = [x_p, y_p]^\top \quad \text{(position only)}
\end{aligned}$$

**プロセスモデル** (連続時間等速度モデルの厳密離散化):

$$F = \begin{bmatrix}
1 & 0 & dt & 0 \\
0 & 1 & 0 & dt \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

**プロセスノイズ** (連続白色雑音加速度モデルを離散化したもの):

$$Q(dt) = \sigma_a^2 \cdot \begin{bmatrix}
dt^4/4 & 0 & dt^3/2 & 0 \\
0 & dt^4/4 & 0 & dt^3/2 \\
dt^3/2 & 0 & dt^2 & 0 \\
0 & dt^3/2 & 0 & dt^2
\end{bmatrix}$$

これは標準的な **Singer モデル** (Bar-Shalom et al., §6.3) であり、$\sigma_a$ は加速度の標準偏差
である。パワースペクトル密度行列は $q(t) = \sigma_a^2 \cdot G G^\top$、
$G = [0, 0, 1, 0;\ 0, 0, 0, 1]^\top$ であり、これを $dt$ にわたって積分したものである。

**観測モデル**:

$$H = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}$$

**カルマン更新** (数値安定性のための Joseph 形式はここでは使用しない。行列が小さいため、
桁落ちが致命的になる可能性は低い):

$$\begin{aligned}
\nu &= z - H \hat{x} \quad \text{(innovation)} \\
S &= H \hat{P} H^\top + R \quad \text{(innovation covariance)} \\
K &= \hat{P} H^\top S^{-1} \quad \text{(Kalman gain)} \\
x &\leftarrow \hat{x} + K \nu \\
P &\leftarrow (I - K H) \hat{P}
\end{aligned}$$

更新処理は、IMM の重み付けに用いる**イノベーション尤度**も返す。

$$L(z \mid \hat{x}, \hat{P}, R) = \mathcal{N}(\nu; 0, S) = \exp(-\tfrac{1}{2} \nu^\top S^{-1} \nu) / (2\pi |S|^{1/2})$$

### 7.2 Interacting Multiple Model (IMM) フィルタ

IMM (Blom & Bar-Shalom, 1988) は、プロセスノイズの異なる 2 つの CV モデルを並列に保持する。

| モデル | $\sigma_a$ | 挙動 |
|---|---|---|
| Cruise | 0.5 m/s² | 高速道路での滑らかな巡航 |
| Manoeuvre | 6.0 m/s² | 急制動または急転舵 |

IMM のモデル確率ベクトル $\mu = [\mu_0, \mu_1]^\top$ (総和 1) は、現在どちらのモデルが有効かを
表す。モード遷移はマルコフ遷移行列に従う。

$$\Pi = \begin{bmatrix}
p_{stay} & 1 - p_{stay} \\
1 - p_{stay} & p_{stay}
\end{bmatrix}, \quad p_{stay} = 0.95$$

**IMM の 1 サイクル** (タイムステップごと):

**ステップ 1 — 相互作用 (混合)。** 予測モデル確率を計算する。

$$\bar{c}_j = \sum_i \Pi_{ij} \mu_i \quad \text{(normaliser for mode } j\text{'s mixing)}$$

モデル $j$ に対する混合初期条件を計算する。

$$\begin{aligned}
\hat{x}^0_j &= \sum_i (\Pi_{ij} \mu_i / \bar{c}_j)\, x_i \\
\hat{P}^0_j &= \sum_i (\Pi_{ij} \mu_i / \bar{c}_j) \big[ P_i + (x_i - \hat{x}^0_j)(x_i - \hat{x}^0_j)^\top \big]
\end{aligned}$$

**ステップ 2 — モード条件付き予測。** 各フィルタ $j$ が $(\hat{x}^0_j, \hat{P}^0_j)$ から予測する。

**ステップ 3 — モード条件付き更新。** 各フィルタ $j$ が観測 $z$ で更新し、尤度
$L_j = L(z \mid \text{model } j)$ を返す。

**ステップ 4 — モデル確率の更新。**

$$\mu_j(\text{new}) = L_j \bar{c}_j / \sum_k L_k \bar{c}_k$$

**ステップ 5 — 融合推定。**

$$\begin{aligned}
\hat{x}_{IMM} &= \sum_j \mu_j \hat{x}_j \\
P_{IMM} &= \sum_j \mu_j \big[ P_j + (\hat{x}_j - \hat{x}_{IMM})(\hat{x}_j - \hat{x}_{IMM})^\top \big]
\end{aligned}$$

**マヌーバ確率 $\mu_1$** それ自体が有用な出力である。これは対象エージェントがどれだけ激しい
機動を行っているかを定量化し、下流のコスト関数のゲーティングに利用できる (例: プランナが
$\mu_1$ の高いエージェントへの接近をより強く罰する)。

### 7.3 多物体追跡 (MOT)

`MultiObjectTracker` は `Track` オブジェクトのベクタをラップする。各 `Track` は 1 つの IMM
フィルタとライフサイクルカウンタ (`hits`, `misses`, `confirmed`) を保持する。

**トラッカの 1 サイクル:**

1. **予測。** すべてのトラックが `IMM::predict(dt)` を実行する。
2. **対応づけ。** ゲート $\gamma = 9.21$ 以内にある (トラック, 検出) の全ペアについて
   `(d², track_idx, det_idx)` のリストを構築する。昇順ソートし、$O(n \log n)$ で貪欲に割り当てる。
   最小の $d^2$ をもつペアが最初にマッチし、両インデックスはその時点でロックされる。
3. **更新。** マッチしたトラックは `IMM::update(z, R)` を実行する。マッチしなかったトラックは
   `misses` をインクリメントする。
4. **生成。** マッチしなかった検出は暫定トラックを生成する。速度は `v_prior` が利用可能なら
   (Radar のドップラーから) それで初期化し、なければゼロとする。
5. **削除・確定。** $misses > max\_miss = 5$ のトラックは削除する。$hits \ge min\_hits = 3$ の
   トラックは `confirmed` に昇格させる。

**マハラノビス・ゲート** $d^2 = \nu^\top S^{-1} \nu < \gamma$ は、センサノイズ水準を超えた誤対応を
防止する。$\gamma = 9.21$ は、真のガウスモデルのもとで 99% の包含率に対応する。ゲート外に落ちる
1% の真の検出については、新規トラック生成によって処理される。

---

## 8. モーションプランニング

### 8.1 アーキテクチャ: 挙動計画と軌道計画の分離

プランナは古典的な**分離型 (decoupled)** 構造 (Paden et al., 2016) に従う。

1. **縦方向計画** — IDM が滑らかな速度プロファイルを生成する。
2. **横方向計画** — smoothstep が滑らかなレーンチェンジプロファイルを生成する。
3. **マルチレーン探索** — 各候補レーンについて両プロファイルを評価し、コストと安全棄却により
   最良のものを選択する。

この分離により、縦横同時最適化における組合せ爆発を回避しつつ、車両追従とレーンチェンジの
双方を計画できる表現力を保っている。

### 8.2 縦方向: Intelligent Driver Model (IDM)

IDM (Treiber et al., 2000) は、自車速度 $v$、先行車とのギャップ $s$、先行車速度 $v_{lead}$ から
滑らかな加速度を生成する。

$$\begin{aligned}
s^*(v, \Delta v) &= s_0 + \max\!\big(0,\ v T + v \Delta v / (2 \sqrt{a b})\big) \\
a_{IDM} &= a \big[1 - (v/v_{des})^4 - (s^*/s)^2\big]
\end{aligned}$$

パラメータ: $a = 1.5$ m/s² (最大加速度)、$b = 2.0$ m/s² (快適減速度)、$T = 1.5$ s (目標車頭時間)、
$s_0 = 5.0$ m (最小停止時車間)、$v_{des} = 13.0$ m/s (目標速度)。IDM はプランニングホライズンに
わたって連続的な速度プロファイルを生成する。

$$\begin{aligned}
v[k+1] &= \max(0,\ v[k] + a_{IDM}(v[k], s[k], v_{lead}[k]) \cdot dt) \\
x[k+1] &= x[k] + v[k] \cdot dt
\end{aligned}$$

各候補レーンについて、各ステップ $k$ で同一レーン内の最も近い予測先行車を特定する。先行車が
存在しない場合は自由走行モデル ($s \to \infty$) を用いる。

**なぜ単純な PID ではなく IDM か。** IDM は社会力モデルを本質的に内包している。`s*` が速度と
ともに増加する滑らかな目標ギャップを生成するため、追従状態と自由流状態の境界でストップ&ゴーの
チャタリングが発生しない。目標速度固定の PID 速度制御器では、このレジーム遷移をモデル化できない。

### 8.3 横方向: smoothstep レーンチェンジプロファイル

横方向プロファイルは、現在レーン $y_0$ と目標レーン $y_{target}$ の間を滑らかに補間する。

$$\begin{aligned}
\sigma(t; a, b) &= \operatorname{clamp}((t-a)/(b-a),\ 0,\ 1) \\
y(t) &= y_0 + (y_{target} - y_0) \cdot [ 3\sigma^2 - 2\sigma^3 ] \quad \text{(smoothstep, } C^1 \text{ continuous)}
\end{aligned}$$

$t_{change} = 3.0$ s はレーンチェンジ所要時間である。smoothstep は $t = a$ および $t = b$ で
一階微分がゼロとなるため、横方向のジャークが有界となる (機動の開始時・終了時にインパルスが
発生しない)。smoothstep が含意する横方向曲率 $\kappa = \ddot{y} / (1 + \dot{y}^2)^{3/2}$ が、
その後コントローラで使用される。

### 8.4 マルチレーン候補評価

各候補レーン $y_{target} \in \{0.0, \text{LANE}\}$ について、軌道 $(x[k], y[k], v[k])$ を生成し
スコアリングする。コスト関数は次のとおりである。

$$\begin{aligned}
\text{cost} = {}& w_1\, \mathbb{E}[\lVert v - v_{des} \rVert^2] && \text{(speed keeping)} \\
& - w_2\, (x[M] - x[0]) && \text{(progress reward, negative = maximise)} \\
& + w_3\, \max_k(v[k]^2 |\kappa[k]|) && \text{(comfort: max lateral acceleration)} \\
& + w_4\, [\text{target} \ne \text{ego\_lane}] && \text{(lane-change cost)} \\
& + w_5\, |\text{target\_lane}| && \text{(right-lane preference, positive } y = \text{ left)} \\
& + w_6 / \max(\text{min\_clear}, 10^{-3}) && \text{(margin penalty, soft)}
\end{aligned}$$

$$w = [1.0, 0.4, 4.0, 6.0, 1.0, 30.0]$$

**安全棄却 (ハード制約)。** ホライズン内の任意のステップにおける、任意の予測エージェントに対する
最小クリアランスを計算する。$\text{min\_clear} < \text{safe\_radius} = 4.5$ m の場合、その候補は
`None` (実行不可能) とする。これはガードレールとは独立した、プランナ自身の衝突回避レイヤである。

**緊急フォールバック。** すべての候補が実行不可能な場合、プランナは現在レーンで $1.5b$ で減速する
軌道を `EMERGENCY_SLOW` のラベル付きで返す。その後はガードレールが RSS 由来の制動により引き継ぐ。

---

## 9. 安全ガードレール — RSS モニタ

ガードレールは、プランナの推論を信頼せずにワールドモデルから安全性を導出し直す**独立モニタ**で
ある。提案されたアクチュエータ指令 $(a, \delta)$ に対して動作し、これを拒否してフェイルセーフ
応答に差し替えることができる。

### 9.1 縦方向 RSS (Shalev-Shwartz et al., 2017)

**最悪ケース距離。** 速度 $v_{ego}$ の自車と、速度 $v_{lead}$ の先行車を考える。最悪ケースでは:

- 先行車が最大減速度 $b_{lead}$ で直ちに制動する。
- 自車は反応時間 $\rho$ のあいだ加速を続け、その後に最小制動能力 $b_{min}$ で制動する。

このとき最小安全車間距離 (停止距離に基づく議論) は次で与えられる。

$$d_{RSS}(v_{ego}, v_{lead}) = v_{ego}\, \rho + \tfrac{1}{2} a_{ego}\, \rho^2 + (v_{ego} + \rho a_{ego})^2 / (2 b_{min}) - v_{lead}^2 / (2 b_{lead})$$

ただし 0 でクランプする。パラメータ: $\rho = 0.4$ s、$a_{ego} = 1.0$ m/s²、$b_{min} = 4.0$ m/s²、
$b_{lead} = 8.0$ m/s²。実際のギャップが $d_{RSS}$ を下回るとガードレールが発火する。

**物理的解釈。** $v_{ego}\, \rho + \tfrac{1}{2} a_{ego}\, \rho^2$ は、自車が反応時間中に加速し
続けながら進む距離である。$(v_{ego} + \rho a_{ego})^2 / (2 b_{min})$ はその後の停止距離である。
$v_{lead}^2 / (2 b_{lead})$ は先行車の停止距離である (先行車も停止するため減算される)。この式は、
上記の最悪ケースモデルのもとで衝突が起きないことを保証する。

### 9.2 衝突余裕時間 (TTC)

$$\text{TTC} = \text{gap} / (v_{ego} - v_{lead}) \quad [\text{if } v_{ego} > v_{lead}]$$

$\text{TTC} < 2.5$ s で発火する。TTC は RSS を補完するチェックである。速度が一致した状態での
過度な接近追従、すなわち (接近速度がゼロのため) RSS 距離条件は満たされているが絶対的なギャップが
危険なほど小さいケースを捕捉する。

### 9.3 カットイン検出のための横方向 RSS

カットインが危険となるのは、横方向と縦方向の安全距離が**同時に**侵害されたときだけである
(Shalev-Shwartz et al., §4)。ガードレールは両方をチェックする。

**横方向 RSS 距離:**

$$d_{lat}(v_{lat}) = \mu + v_{lat}\, \rho + \tfrac{1}{2} a_{lat\_max}\, \rho^2 + (v_{lat} + \rho a_{lat\_max})^2 / (2 b_{lat\_min})$$

ここで $\mu = 0.5$ m はクリアランスバッファ、$a_{lat\_max} = 0.5$ m/s²、
$b_{lat\_min} = 1.0$ m/s² である。

**カットイン判定条件** (すべてが同時に成立する必要がある):

1. エージェントが横方向に接近している: $(y_{agent} - y_{ego}) \cdot vy_{agent} < 0$。
2. 横方向ギャップが $|y_{agent} - y_{ego}| < d_{lat}(|vy_{agent}|)$。
3. 縦方向ギャップが RSS 帯域内: $-L_{veh} \le (x_{agent} - x_{ego}) \le d_{RSS} + L_{veh}$。

2 と 3 が同時に成立するとき、正しい RSS 応答は — 安全な横方向回避が存在しないため —
**縦方向の制動**である。

### 9.4 ラッチ機構

いずれかのチェックが発火したとき:

$$\begin{aligned}
& \text{latch} \leftarrow \text{hold} = 8 \quad \text{(0.8 s at } dt = 0.1 \text{ s)} \\
& \text{response: } a = -b_{emergency} = -6 \text{ m/s}^2, \ \delta = \delta_{previous} \text{ (hold steering)}
\end{aligned}$$

`latch` は各ステップでデクリメントされる。ガードレールは $\text{latch} = 0$ になるまでオーバー
ライドを継続する。これは、RSS 条件が解消した瞬間にオーバーライドを解除した場合に生じる境界での
振動 (自車は境界上でなお接近を続けているため) を防止する。ラッチが満了するころには、自車は
$b_{emergency} \cdot 0.8 \approx 4.8$ m/s だけ減速しており、通常はこれで安全なギャップ余裕を
回復するのに十分である。

---

## 10. 確率的占有予測

占有グリッドは、前方道路の**連続的な確率マップ**を提供する。これはプランナが用いる離散的な軌道
予測の代替として、あるいはそれと併用して使うことができる。

### 10.1 占有モデル

予測された各エージェント軌道 $\{(x_k, y_k)\}_{k=0}^{K}$ は、時間とともに不確かさが増大する
エージェントのフットプリントを表す、軸平行ガウス分布としてスプラット (splat) される。

$$\begin{aligned}
\sigma(k) &= \sigma_0 + \gamma k \quad \text{(uncertainty inflation)} \\
P_i(\text{cell} \mid \text{horizon } k) &= \exp\!\big[-\tfrac{1}{2} \big( (X - x_k)^2 / (car_l + \sigma)^2 + (Y - y_k)^2 / (car_w + \sigma)^2 \big)\big]
\end{aligned}$$

ここで $\sigma_0 = 0.6$ m、$\gamma = 0.06$ m/step、$car_l = 2.2$ m (半長)、$car_w = 0.9$ m
(半幅) である。ガウス分布の広がりはホライズンに対して線形に増大し、予測エージェント位置の
不確かさが増していくことを反映している。

### 10.2 集約: 確率的 OR

複数エージェントは**確率的 OR** (独立事象の和事象) により統合する。

$$P(\text{cell occupied}) = 1 - \prod_i (1 - P_i(\text{cell}))$$

これは各エージェントが独立にセルを占有すると扱うことに相当し、保守的である (エージェント間に
相関がある場合はリスクを過大評価する)。確率的 OR により、複数エージェントが占有する領域で
マップが二重計上なしに 1 へ飽和することが保証される。

### 10.3 下流での利用

- **ソフトコスト** (プランナ): 候補軌道に沿って $P(\text{occupied})$ を積分し、クリアランス項を
  補強する。
- **ハード拒否** (ガードレール): $\max_k P(\text{cell}) > \text{threshold}$ となる計画軌道を
  ブロックする。
- **可視化** (`run_pipeline.py`): 各タイムステップでマップを表示する。

**正しさの保証。** `test_occupancy` テストは以下を検証する。
- 確定エージェントに対し、0.5 s 以内でピーク占有確率 $> 0.8$ となること。
- $P > 0.5$ となるセルの面積が、ホライズン 5 からホライズン 30 にかけて増大すること
  (不確かさインフレーション)。

---

## 11. 検証と数値結果

以下に示す数値はすべて、テストスイート (`pytest tests/ -v`) およびエントリポイントスクリプトに
よって再現される。

| テスト | 指標 | 結果 |
|---|---|---|
| 自転車モデル線形化残差 | $\lVert \Phi(s+\delta s) - (\Phi(s) + A \delta s) \rVert / \lVert \delta s \rVert$ | **$2.3 \times 10^{-6}$** (2 次精度 ✓) |
| LQR 定常横偏差 | ダブルレーンチェンジ定常時の $\lvert e_y \rvert$ | **$8.6 \times 10^{-14}$ m** |
| MPC 速度追従 | 定常時の $\lvert v - v_{des} \rvert$ | **$< 10^{-4}$ m/s** |
| MPC 最大加速度 | 加速中の $\lvert a \rvert$ | **3.00 m/s²** (制限値 = 3.0 ✓) |
| MPC 最大操舵角 | 横方向復帰中の $\lvert \delta \rvert$ | **0.600 rad** (制限値 = 0.6 ✓) |
| IMM 追跡位置誤差 | ウォームアップ (3 s) 後の平均位置 RMSE | **< 0.6 m** |
| IMM 追跡速度誤差 | 平均速度 RMSE | **< 1.2 m/s** |
| IMM マヌーバ検出 | 急制動前後の $p_{man}$ | 有意に上昇 ✓ |
| ガードレール — 正常な IDM | 急制動シナリオでの衝突 | **なし** (安全) |
| ガードレール — 故障プランナ、OFF | 急制動シナリオでの衝突 | **あり** (想定どおり) |
| ガードレール — 故障プランナ、ON | 急制動シナリオでの衝突 | **なし** (捕捉成功) |
| 横方向 RSS — 反応時間 | 縦方向のみの場合との比較 | **遅れなし** (同等以上に早い) |
| 占有 — ピーク | 0.5 s 以内の P(occupied) | **> 0.8** |
| 占有 — インフレーション | ホライズン 5 → 30 における area(P > 0.5) | **単調増加** |

---

## 12. 参考文献

**Vehicle dynamics**

- Rajamani, R. (2012). *Vehicle Dynamics and Control*, 2nd ed. Springer.
- Kong, J. et al. (2015). "Kinematic and dynamic vehicle models for autonomous driving control design." *IEEE IV*.

**Numerical integration**

- Press, W. H. et al. (2007). *Numerical Recipes: The Art of Scientific Computing*, 3rd ed. Cambridge.

**Control**

- Åström, K. J. & Wittenmark, B. (1997). *Computer-Controlled Systems*, 3rd ed. Prentice Hall.
- Anderson, B. D. O. & Moore, J. B. (1989). *Optimal Control: Linear Quadratic Methods*. Prentice Hall.
- Rawlings, J. B., Mayne, D. Q. & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design*, 2nd ed. Nob Hill Publishing.
- Beck, A. & Teboulle, M. (2009). "A fast iterative shrinkage-thresholding algorithm for linear inverse problems." *SIAM Journal on Imaging Sciences*, 2(1), 183–202.

**Estimation and tracking**

- Bar-Shalom, Y., Li, X.-R. & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*. Wiley.
- Blom, H. A. P. & Bar-Shalom, Y. (1988). "The interacting multiple model algorithm for systems with Markovian switching coefficients." *IEEE Transactions on Automatic Control*, 33(8), 780–783.

**Planning**

- Treiber, M., Hennecke, A. & Helbing, D. (2000). "Congested traffic states in empirical observations and microscopic simulations." *Physical Review E*, 62(2), 1805.
- Paden, B. et al. (2016). "A survey of motion planning and control techniques for self-driving urban vehicles." *IEEE Transactions on Intelligent Vehicles*, 1(1), 33–55.

**Safety**

- Shalev-Shwartz, S., Shammah, S. & Shashua, A. (2017). "On a formal model of safe and scalable self-driving cars." *arXiv:1708.06374*.
- ISO 26262:2018. *Road vehicles — Functional safety*.
- IEC 61508:2010. *Functional Safety of E/E/PE Safety-related Systems*.

**Sensor fusion**

- Thrun, S., Burgard, W. & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
- Gustafsson, F. (2010). *Statistical Sensor Fusion*. Studentlitteratur.
