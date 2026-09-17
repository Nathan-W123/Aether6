#!/usr/bin/env bash
# Build, test and reproduce every result and figure in the repository.
# Usage: ./scripts/run_all.sh [build-dir]
set -euo pipefail

BUILD_DIR="${1:-build}"
JOBS="$(nproc 2>/dev/null || echo 4)"
BIN="${BUILD_DIR}/bin"

step() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

step "Configuring (${BUILD_DIR}, Release)"
cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release

step "Building with ${JOBS} jobs"
cmake --build "${BUILD_DIR}" -j "${JOBS}"

step "Running the unit-test suite"
"${BIN}/aether_tests" 

step "Trim, linearisation and modal analysis"
"${BIN}/aether_trim" configs/scenarios/nominal.yaml --output results/linear --sweep 16:34:19

step "Closed-loop scenarios"
for scenario in nominal ideal nominal_pid wind_disturbance; do
  "${BIN}/aether_sim" "configs/scenarios/${scenario}.yaml"
done

step "Integrator accuracy and cost study"
"${BIN}/aether_integrators" configs/scenarios/nominal.yaml --output results/integrators

step "Monte-Carlo campaign"
"${BIN}/aether_mc" configs/scenarios/monte_carlo.yaml

step "Figures"
python3 python/scripts/make_figures.py

step "Done - see results/ and results/figures/"
