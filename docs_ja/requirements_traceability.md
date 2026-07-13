# Requirements Traceability

> English version: [`../docs_en/requirements_traceability.md`](../docs_en/requirements_traceability.md)

HARA (`hara.md`) から導出した安全目標 → 技術要求 → 実装コンポーネントの対応表。
例示目的のトレーサビリティ文書であり、実際の安全審査の成果物ではない。

チェーン: 安全目標 (SG) → 要求 (REQ) → 設計要素 → 検証

## マトリクス

| Req ID     | 安全目標 | 要求                                              | 設計要素 (コンポーネント)                         | 検証                                    |
|------------|---------|--------------------------------------------------|--------------------------------------------------|-----------------------------------------|
| REQ-PER-01 | SG1     | 障害物検出精度 $\ge 90$%                          | `perception.py`, `tracking.hpp`                  | `tests/test_modules.py` (perception)    |
| REQ-SAF-01 | SG1     | 停止距離保証: 既知障害物手前で必ず停車            | `guardrail.py` (RSS/TTC), `speed_governor.hpp`   | `tests/test_safety.py` (case 1, 2)     |
| REQ-SAF-02 | SG2     | 横断障害物予測: $T_{horizon}$ 内に衝突を事前回避  | `speed_governor.hpp` (定速予測)                  | `tests/test_safety.py` (case 2)        |
| REQ-SAF-03 | SG2     | 緊急ブレーキ $\le 0.5$ s 応答                     | `guardrail.py` (veto+brake)                      | `tests/test_modules.py` (guardrail)    |
| REQ-CTL-01 | SG3     | 横方向誤差 $< 0.5$ m (路追従)                     | `mpc.hpp`, `lqr.hpp`                             | `tests/test_control.py` (LQR/MPC)     |
| REQ-LOC-01 | SG3     | 自己位置精度 $< 0.3$ m (位置ノイズ $0.3$ m 環境下) | `ekf.hpp` (EKF ローカライザ)                     | `tests/test_localization.py` (case 1)  |
| REQ-SAF-04 | SG4     | RSS 誤検知率 $< 1$% (前方クリア時に停止しない)    | `guardrail.py` (TTC 閾値設定)                    | `tests/test_safety.py` (case 1, 3)    |

## 検証状況 (参考)

- REQ-CTL-01: LQR クロストラック誤差 $3.12\times10^{-31}$ m (直線), MPC 速度維持誤差 $0$ m/s
- REQ-LOC-01: EKF — ノイズなし直進で誤差 $< 0.01$ m (test_localization, case 1)
- REQ-SAF-01: 障害物なし → target_v 変化なし (test_safety, case 1)
- REQ-SAF-02: 正面障害物あり → 速度低下を確認 (test_safety, case 2)

## 制限事項

- 検証はユニット・シミュレーションレベルのみ (HIL・実車不使用)
- ASIL 分解・独立性解析は未実施
- SG2 のトリガー条件 (閉塞・センサ脱落) は未境界化 (SOTIF 課題)
