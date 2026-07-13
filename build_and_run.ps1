#Requires -Version 5.1
<#
.SYNOPSIS
    Build av-stack and launch a simulation (Windows / MSVC)

.PARAMETER SkipBuild
    Skip the build and run the simulation only (use when already built)

.PARAMETER SkipTests
    Skip the C++ / Python test suites

.PARAMETER Demo
    Select the demo to launch:
      "pipeline"     full closed-loop pipeline comparison (guardrail ON/OFF) — default
      "animation"    pure-Python animation demo (hard brake + cut-in)
      "acc"          ACC car-following demo (follow a lead at 8 m/s)
      "avoidance"    lane-change avoidance demo (overtake a slow obstacle on the left)
      "lead_brake"   moderate lead braking (4 m/s^2); the nominal planner copes unaided
      "mixed"        mixed traffic (slow lead + faster car in the left lane), IMM tracking
      "cut_in"       cut-in demo (left-lane neighbour merges in front of a faulty planner)
      "safety_stop"  functional-safety stop demo (lead hard-brakes + left lane blocked -> guardrail stops)

    The C++ scenarios all reach the guardrail through the same pipeline; the demos
    differ in the scenario and in whether the planner is nominal or faulty.

.EXAMPLE
    .\build_and_run.ps1
    .\build_and_run.ps1 -SkipBuild
    .\build_and_run.ps1 -SkipBuild -Demo acc
    .\build_and_run.ps1 -SkipBuild -Demo avoidance
    .\build_and_run.ps1 -SkipBuild -Demo lead_brake
    .\build_and_run.ps1 -SkipBuild -Demo mixed
    .\build_and_run.ps1 -SkipBuild -Demo cut_in
    .\build_and_run.ps1 -SkipBuild -Demo safety_stop
#>
param(
    [switch]$SkipBuild,
    [switch]$SkipTests,
    [ValidateSet("animation", "pipeline", "acc", "avoidance",
                 "lead_brake", "mixed", "cut_in", "safety_stop")]
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
        Write-Host "ERROR: '$name' not found. Install it and add it to PATH." -ForegroundColor Red
        exit 1
    }
}

# -- Prerequisites --------------------------------------------------------
Assert-Command "cmake"
Assert-Command "python"

if (-not $SkipBuild) {
    # -- Python dependencies ----------------------------------------------
    Write-Header "Installing Python dependencies"
    python -m pip install pybind11 numpy matplotlib pytest --quiet

    # -- CMake configure --------------------------------------------------
    Write-Header "CMake configure"
    $pybind11Dir = python -m pybind11 --cmakedir
    cmake -S $Root -B "$Root\build" `
        -DCMAKE_BUILD_TYPE=Release `
        "-Dpybind11_DIR=$pybind11Dir"

    if ($LASTEXITCODE -ne 0) {
        Write-Host "CMake configure failed." -ForegroundColor Red; exit 1
    }

    # -- Build ------------------------------------------------------------
    Write-Header "Build (Release)"
    $jobs = (Get-CimInstance Win32_Processor).NumberOfLogicalProcessors
    cmake --build "$Root\build" --config Release --parallel $jobs

    if ($LASTEXITCODE -ne 0) {
        Write-Host "Build failed." -ForegroundColor Red; exit 1
    }
}

if (-not $SkipTests) {
    # -- C++ self-test ----------------------------------------------------
    Write-Header "C++ control test"
    & "$Root\build\Release\control_test.exe"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "C++ tests failed." -ForegroundColor Red; exit 1
    }

    # -- Python test suite ------------------------------------------------
    Write-Header "Python tests"
    $env:PYTHONPATH = "$Root\python;$Root\build\Release;$Root\build"
    python -m pytest "$Root\tests" -v
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Python tests failed." -ForegroundColor Red; exit 1
    }
}

# -- Launch the simulation ------------------------------------------------
$env:PYTHONPATH = "$Root\python;$Root\build\Release;$Root\build"
$env:PYTHONIOENCODING = "utf-8"

switch ($Demo) {
    "animation" {
        Write-Header "Launching simulation - animation_demo.py"
        Write-Host "Scenario: lead vehicle hard-brakes + cut-in -> guardrail check" -ForegroundColor Yellow
        python "$Root\python\animation_demo.py"
    }
    "pipeline" {
        Write-Header "Launching simulation - run_pipeline.py"
        Write-Host "Scenario: full modular pipeline, closed loop (guardrail ON/OFF comparison)" -ForegroundColor Yellow
        python "$Root\python\run_pipeline.py"
    }
    "acc" {
        Write-Header "Demo: ACC car-following"
        Write-Host "Scenario: follow a lead vehicle at 8 m/s with ACC (v_des = 13 m/s)" -ForegroundColor Yellow
        Write-Host "Expected: FOLLOW/SLOW - stable headway, guardrail never fires" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario acc
    }
    "avoidance" {
        Write-Header "Demo: lane-change avoidance"
        Write-Host "Scenario: slow obstacle (3 m/s) in the ego lane, left lane is clear" -ForegroundColor Yellow
        Write-Host "Expected: CHANGE_LANE -> change into the left lane and overtake -> CRUISE" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario avoidance
    }
    "lead_brake" {
        Write-Header "Demo: moderate lead braking"
        Write-Host "Scenario: lead vehicle brakes at 4 m/s^2 at t = 3 s (nominal planner)" -ForegroundColor Yellow
        Write-Host "Expected: FOLLOW/SLOW then CHANGE_LANE to overtake - guardrail never fires" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario lead_brake
    }
    "mixed" {
        Write-Header "Demo: mixed traffic"
        Write-Host "Scenario: slow lead (7 m/s) in the ego lane, faster car (13 m/s) in the left lane" -ForegroundColor Yellow
        Write-Host "Expected: IMM tracks both agents; ego settles into FOLLOW/SLOW - guardrail never fires" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario mixed
    }
    "cut_in" {
        Write-Header "Demo: cut-in"
        Write-Host "Scenario: a left-lane neighbour merges in front of the ego (t 0.5-3.5 s)" -ForegroundColor Yellow
        Write-Host "Expected: the faulty planner ignores the merge -> RSS guardrail brakes" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario cut_in
    }
    "safety_stop" {
        Write-Header "Demo: functional-safety stop"
        Write-Host "Scenario: lead hard-brakes (8 m/s^2) + vehicle alongside in the left lane -> nowhere to go" -ForegroundColor Yellow
        Write-Host "Expected: EMERGENCY_SLOW -> RSS guardrail brakes hard (-6 m/s^2) -> stop" -ForegroundColor Cyan
        python "$Root\python\run_pipeline.py" --scenario safety_stop
    }
}

if ($LASTEXITCODE -ne 0) {
    Write-Host "The simulation exited with an error." -ForegroundColor Red; exit 1
}

Write-Host ""
Write-Host "=== Done ===" -ForegroundColor Green
