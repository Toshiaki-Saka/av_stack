#Requires -Version 5.1
<#
.SYNOPSIS
    av-stack をビルドしてシミュレーションを起動する (Windows / MSVC 用)

.PARAMETER SkipBuild
    既にビルド済みの場合にビルドをスキップしてシミュレーションのみ実行

.PARAMETER SkipTests
    C++ / Python テストをスキップ

.PARAMETER Demo
    起動するデモを選択:
      "pipeline"     フルパイプライン閉ループ比較 (guardrail ON/OFF) — デフォルト
      "animation"    純Pythonアニメーションデモ (急制動 + 割り込み)
      "acc"          ACC追従デモ (先行車 8 m/s に追従)
      "avoidance"    車線変更回避デモ (低速障害物を左車線で回避)
      "safety_stop"  機能安全停車デモ (前方急制動 + 左車線封鎖 → guardrail 停車)

.EXAMPLE
    .\build_and_run.ps1
    .\build_and_run.ps1 -SkipBuild
    .\build_and_run.ps1 -SkipBuild -Demo acc
    .\build_and_run.ps1 -SkipBuild -Demo avoidance
    .\build_and_run.ps1 -SkipBuild -Demo safety_stop
#>
param(
    [switch]$SkipBuild,
    [switch]$SkipTests,
    [ValidateSet("animation", "pipeline", "acc", "avoidance", "safety_stop")]
    [string]$Demo = "pipeline"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$Root = $PSScriptRoot

function Write-Header([string]$msg) {
    Write-Host ""
    Write-Host "=== $msg ===" -ForegroundColor Cyan
}

function Assert-Command([string]$name) {
    if (-not (Get-Command $name -ErrorAction SilentlyContinue)) {
        Write-Host "ERROR: '$name' が見つかりません。インストールしてPATHを通してください。" -ForegroundColor Red
        exit 1
    }
}

# ── 前提コマンド確認 ──────────────────────────────────────────────
Assert-Command "cmake"
Assert-Command "python"

if (-not $SkipBuild) {
    # ── Python 依存パッケージ ──────────────────────────────────────
    Write-Header "依存パッケージのインストール"
    python -m pip install pybind11 numpy matplotlib pytest --quiet

    # ── CMake configure ───────────────────────────────────────────
    Write-Header "CMake configure"
    $pybind11Dir = python -m pybind11 --cmakedir
    cmake -S $Root -B "$Root\build" `
        -DCMAKE_BUILD_TYPE=Release `
        "-Dpybind11_DIR=$pybind11Dir"

    if ($LASTEXITCODE -ne 0) {
        Write-Host "CMake configure に失敗しました。" -ForegroundColor Red; exit 1
    }

    # ── ビルド ────────────────────────────────────────────────────
    Write-Header "ビルド (Release)"
    $jobs = (Get-CimInstance Win32_Processor).NumberOfLogicalProcessors
    cmake --build "$Root\build" --config Release --parallel $jobs

    if ($LASTEXITCODE -ne 0) {
        Write-Host "ビルドに失敗しました。" -ForegroundColor Red; exit 1
    }
}

if (-not $SkipTests) {
    # ── C++ 自己テスト ────────────────────────────────────────────
    Write-Header "C++ control test"
    & "$Root\build\Release\control_test.exe"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "C++ テストに失敗しました。" -ForegroundColor Red; exit 1
    }

    # ── Python テストスイート ─────────────────────────────────────
    Write-Header "Python tests"
    $env:PYTHONPATH = "$Root\python;$Root\build\Release;$Root\build"
    python -m pytest "$Root\tests" -v
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Python テストに失敗しました。" -ForegroundColor Red; exit 1
    }
}

# ── シミュレーション起動 ──────────────────────────────────────────
$env:PYTHONPATH = "$Root\python;$Root\build\Release;$Root\build"
$env:PYTHONIOENCODING = "utf-8"

switch ($Demo) {
    "animation" {
        Write-Header "シミュレーション起動 — animation_demo.py"
        Write-Host "シナリオ: 前方車急制動 + 割り込み → guardrail 動作確認" -ForegroundColor Yellow
        python "$Root\python\animation_demo.py"
    }
    "pipeline" {
        Write-Header "シミュレーション起動 — run_pipeline.py"
        Write-Host "シナリオ: フルモジュラーパイプライン閉ループ (guardrail ON/OFF 比較)" -ForegroundColor Yellow
        python "$Root\python\run_pipeline.py"
    }
    "acc" {
        Write-Header "デモ: ACC追従"
        Write-Host "シナリオ: 先行車 8 m/s に ACC で追従 (v_des = 13 m/s)" -ForegroundColor Yellow
        Write-Host "期待動作: FOLLOW/SLOW — 安定した車間維持、guardrail 発動なし" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario acc
    }
    "avoidance" {
        Write-Header "デモ: 車線変更回避"
        Write-Host "シナリオ: 低速障害物 (3 m/s) が自車線に存在、左車線は空き" -ForegroundColor Yellow
        Write-Host "期待動作: CHANGE_LANE → 左車線に車線変更して追い越し → CRUISE" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario avoidance
    }
    "safety_stop" {
        Write-Header "デモ: 機能安全停車"
        Write-Host "シナリオ: 前方車が急制動 (8 m/s²) + 左車線に並走車 → 逃げ場なし" -ForegroundColor Yellow
        Write-Host "期待動作: EMERGENCY_SLOW → RSS guardrail が緊急制動 (-6 m/s²) → 停車" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario safety_stop
    }
}

if ($LASTEXITCODE -ne 0) {
    Write-Host "シミュレーションがエラーで終了しました。" -ForegroundColor Red; exit 1
}

Write-Host ""
Write-Host "=== 完了 ===" -ForegroundColor Green
