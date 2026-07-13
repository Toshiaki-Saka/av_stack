# Requirements Traceability

> English version: [`../docs_en/requirements_traceability.md`](../docs_en/requirements_traceability.md)

HARA (`hara.md`) から導出した安全目標 → 技術要求 → 実装コンポーネントの対応表。
例示目的のトレーサビリティ文書であり、実際の安全審査の成果物ではない。

チェーン: 安全目標 (SG) → 要求 (REQ) → 設計要素 → 検証

**設計要素は本番パスを指す。** パイプラインのアルゴリズムはすべて C++ (`cpp/include/**`)
として出荷され、pybind11 経由で公開される。`python/*.py` の同等物は参照実装であり、実際に
動作するものでは**ない**。したがって要求はヘッダにトレースする。この区別が論証上重要である
理由は [`SAFETY_CASE.md`](SAFETY_CASE.md) の残留リスク R3 を参照。

## マトリクス

| Req ID     | 安全目標 | 要求                                              | 設計要素 (本番コンポーネント)                     | 検証                                    |
|------------|---------|--------------------------------------------------|--------------------------------------------------|-----------------------------------------|
| REQ-PER-01 | SG1     | 障害物検出精度 $\ge 90$%                          | `perception/fusion.hpp`, `world/tracking.hpp`    | `tests/test_modules.py` (perception)    |
| REQ-SAF-01 | SG1     | 停止距離保証: 既知障害物手前で必ず停車            | `safety/guardrail.hpp` (RSS/TTC), `ad/safety/speed_governor.hpp` | `tests/test_safety.py` (case 1, 2)     |
| REQ-SAF-02 | SG2     | 横断障害物予測: $T_{horizon}$ 内に衝突を事前回避  | `ad/safety/speed_governor.hpp` (定速予測)        | `tests/test_safety.py` (case 2)        |
| REQ-SAF-03 | SG2     | 緊急ブレーキ $\le 0.5$ s 応答                     | `safety/guardrail.hpp` (veto+brake)              | `tests/test_modules.py` (guardrail)    |
| REQ-SAF-05 | SG2     | カットイン検知は隣接車が自車線に入る前に発火すること | `safety/guardrail.hpp::_cut_in_threat` (横方向 RSS) | `tests/test_safety.py::test_guardrail_lateral_rss_matches_documented_formula`, `::test_guardrail_lateral_rss_exceeds_half_lane`, `::test_guardrail_cut_in_fires_before_lane_encroachment`, `tests/test_modules.py::test_lateral_rss` |
| REQ-CTL-01 | SG3     | 横方向誤差 $< 0.5$ m (路追従)                     | `control/mpc.hpp`, `control/lqr.hpp`             | `tests/test_control.py` (LQR/MPC)     |
| REQ-LOC-01 | SG3     | 自己位置精度 $< 0.3$ m (位置ノイズ $0.3$ m 環境下) | `ad/localization/ekf_localizer.hpp`              | `tests/test_localization.py` (case 1)  |
| REQ-SAF-04 | SG4     | RSS 誤検知率 $< 1$% (前方クリア時に停止しない)    | `safety/guardrail.hpp` (TTC 閾値、ゴーストトラック除外、接近方向チェック) | `tests/test_safety.py` (case 1, 3), `::test_guardrail_no_cut_in_for_departing_neighbour`, `::test_guardrail_ignores_ghost_track_lateral_speed` |

## 検証状況 (参考)

- REQ-CTL-01: LQR クロストラック誤差 $3.12\times10^{-31}$ m (直線), MPC 速度維持誤差 $0$ m/s
- REQ-LOC-01: EKF — ノイズなし直進で誤差 $< 0.01$ m (test_localization, case 1)
- REQ-SAF-01: 障害物なし → target_v 変化なし (test_safety, case 1)
- REQ-SAF-02: 正面障害物あり → 速度低下を確認 (test_safety, case 2)
- REQ-SAF-05: `scenario_cut_in` にプランナのフォールトを注入した状態で、横方向チェックは 1.2 s に反応する (縦方向単独では 1.7 s)。最接近距離は 13.08 m (同 11.16 m)。$v_y = 2$ m/s において $d_{lat} = 3.82$ m であり、$\text{LANE}/2 = 1.75$ m を上回るため、隣接車がまだ隣車線にいる段階で発火する。

## 制限事項

- 検証はユニット・シミュレーションレベルのみ (HIL・実車不使用)
- ASIL 分解・独立性解析は未実施
- SG2 のトリガー条件 (閉塞・センサ脱落) は未境界化 (SOTIF 課題)
- REQ-SAF-04 は率として記述されているが、検証は個別ケース (離脱していく隣接車とゴーストトラックで制動しないこと) にとどまる。シナリオ母集団に対する誤検知率は測定しておらず、$< 1$% は目標値であってエビデンスではない。
- ガードレールは `safety/guardrail.hpp` と `python/guardrail.py` の 2 箇所に実装されており、過去に乖離が生じている。[`SAFETY_CASE.md`](SAFETY_CASE.md) の残留リスク R3 を参照
