# av-stack（日本語概要）

古典的なモジュラー自動運転スタックの研究・教育向け実装です。センサー・知覚・
ワールドモデル（追跡）・計画・制御という各層を独立したモジュールとして実装し、
**RSS（Responsibility-Sensitive Safety）** に基づく独立安全ガードレールが
プランナーの出力を監視します。

外部依存なし（ROS 不要・外部データセット不要・GPU 不要）で完結します。
合成の多車線走行シミュレータがシナリオと閉ループ試験環境を提供します。

日本語ドキュメントは [`docs_ja/`](docs_ja/) に、英語版は [`docs_en/`](docs_en/) に
揃えてあります。設計解説は [`ARCHITECTURE.md`](docs_ja/ARCHITECTURE.md)、
安全論証は [`SAFETY_CASE.md`](docs_ja/SAFETY_CASE.md)、理論と実装の詳細は
[`TECHNICAL.md`](docs_ja/TECHNICAL.md)、ハザード分析は [`hara.md`](docs_ja/hara.md)、
要求トレーサビリティは
[`requirements_traceability.md`](docs_ja/requirements_traceability.md) を参照してください。
全結果を含む英語版の概要は [`README.md`](README.md) にあります。

```
センサー群 ──► 知覚（融合） ──► 追跡器 ──► プランナー ──► 制御器 ──► 車両
 LiDAR       逆共分散融合     IMM      IDM+車線   PID/LQR/MPC   運動学的
 Radar                                変更スコア               二輪モデル
 Camera
                                 ▲                               │
                        RSS+TTC ガードレール ◄─────────────────────┘
```

## 構成（C++-first）

本スタックは **C++-first** です。パイプラインの全アルゴリズム（制御・追跡・
センサー・知覚・計画・安全ガードレール・占有格子）は `cpp/include/` 配下の C++17
ヘッダで実装し、pybind11 で Python へ公開します（`control_cpp`, `world_cpp`,
`perception_cpp`, `planning_cpp`, `guardrail_cpp`, `occupancy_cpp`, `sensors_cpp`,
`scenario_cpp`, `localization_cpp`, `safety_cpp`, `hybrid_cpp`）。`python/*.py` は
可視化・デモ・シナリオ駆動の層です。

- **車両モデル・制御（C++17）** — RK4 積分の運動学的二輪モデル、PID（アンチ
  ワインドアップ）、離散 LQR（DARE 後退反復）、LTV-MPC（FISTA QP ソルバー）。

- **センサー・知覚** — LiDAR / Radar / Camera の物体レベルノイズモデル（それぞれ
  異方性共分散 $R$ を出力）、情報形式による最尤センサー融合、マハラノビス距離
  ゲーティング。Radar Doppler で速度先行値を取得。

- **追跡** — **IMM（Interacting Multiple Model）** 追跡器。低ノイズの cruise
  モデルと高ノイズの manoeuvre モデル（定速 CV）を混合し、巡航時は推定を
  引き締めつつ急ブレーキ・割り込み時には素早く追従。実体は
  `cpp/include/world/tracking.hpp` の `struct IMM`。

- **計画** — IDM（Intelligent Driver Model）による縦方向プロファイル生成、
  smoothstep 横方向プロファイル、速度維持・進捗・快適性・安全距離に基づく
  コスト関数でトラジェクトリを選択。安全半径内に進入する候補は棄却。

- **安全ガードレール（RSS + TTC）** — プランナーとは独立した doer-checker。
  縦方向 RSS 安全距離、TTC、横方向 RSS（割り込み検知）の 3 重チェック。
  いずれかが発火すると緊急ブレーキにラッチし、hold ステップ間保持。

## 主な検証済み結果

| 項目 | 結果 |
|---|---|
| 自転車モデル線形化残差 | $O(\varepsilon^2) = 2.3\times10^{-6}$（非線形ステップと整合） |
| LQR 横偏差残差（8 s 後） | $8.6\times10^{-14}$ m（実質ゼロ） |
| MPC 速度追従 | $v = 12.00$ m/s（目標 12）、$\max\lvert a\rvert = 3.00$（制限 3.0） |
| MPC 横方向回復 | $y = 0.000$ m（目標 0）、$\max\lvert\delta\rvert = 0.600$（制限 0.6） |
| 追跡器定常誤差 | 位置 $< 0.6$ m、速度 $< 0.4$ m/s |
| ガードレール（急ブレーキシナリオ） | 追突 0 回（ガードレールなしでは衝突） |
| ガードレール（割り込みシナリオ） | 横方向 RSS が横重複の約 1.5 s 前に発火 |
| Python テストスイート | 28 合格（`pytest tests/`、5 ファイル・C++ モジュールを使用） |

## クイックスタート

**Windows (MSVC) — ワンコマンド。** リポジトリ直下の `build_and_run.ps1` が、Python 依存
パッケージのインストール → C++ コアの CMake configure・ビルド → C++/Python テスト実行 →
デモ起動までを一括で実行します。

```powershell
.\build_and_run.ps1                          # ビルド + テスト + フルパイプラインdemo（既定）
.\build_and_run.ps1 -SkipBuild               # ビルドを省略してdemoのみ実行
.\build_and_run.ps1 -SkipBuild -SkipTests    # ビルドとテストを省略
.\build_and_run.ps1 -Demo acc                # demoを選択（下記参照）
```

`-Demo` でシナリオを選択します。C++ 側の全シナリオを起動できます。

| `-Demo` | シナリオ | プランナー | 見どころ |
|---|---|---|---|
| `pipeline`（既定） | 急制動 8 m/s² | 故障 | guardrail ON/OFF 比較 — なしでは衝突、ありでは安全停止 |
| `animation` | 急制動 + 割り込み | — | 純Pythonリアルタイムアニメーション（C++ ビルド不要） |
| `lead_brake` | 中程度の制動 4 m/s² | 正常 | 追従後に追い越し。guardrail は発火しない |
| `mixed` | 低速先行車 + 左車線の高速車 | 正常 | IMM が2台を追跡、自車は低速車に追従 |
| `acc` | 先行車が 8 m/s で巡航 | 正常 | ACC 追従。guardrail は安全網として待機 |
| `avoidance` | 低速障害物 3 m/s | 正常 | `CHANGE_LANE` で左車線へ追い越し、`CRUISE` に復帰 |
| `cut_in` | 左隣レーンからの割り込み | 故障 | プランナーが割り込みを無視、guardrail が制動 |
| `safety_stop` | 急制動 + 左車線封鎖 | 故障 | 逃げ場なし。RSS guardrail が緊急停車 |

**プラットフォーム非依存 — ビルド不要:**

```bash
pip install -r requirements.txt
python python/animation_demo.py   # リアルタイムアニメーション（C++ ビルド不要）
```

C++ コアをビルドする場合（Linux / macOS）:

```bash
pip install pybind11
./build.sh
pytest tests/ -v
```

Windows (MSVC) の場合:

```bat
cmake -S . -B build
cmake --build build --config Release
build\Release\control_test.exe
```

ライセンスは MIT（[`LICENSE`](LICENSE)）。コードはすべて独自実装、シナリオは
合成生成で、第三者の著作物・データセットを含みません。
