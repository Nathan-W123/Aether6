# =============================================================================================
# Aether-6 convenience targets.
#
#   make configure     configure the CMake build (Release)
#   make build         build the library, applications and tests
#   make test          run the full Catch2 suite (make ctest runs it through CTest)
#   make run           run the nominal closed-loop mission
#   make trim          compute trim, linearise, print the modes, export the model
#   make monte-carlo   run the Monte-Carlo campaign
#   make integrators   run the integrator accuracy/cost study
#   make figures       generate every figure (including the animated GIF)
#   make run-all       everything above, in order
#   make clean         remove the build directory
#   make clean-results remove generated results (keeps the committed figures)
# =============================================================================================

BUILD_DIR      ?= build
BUILD_TYPE     ?= Release
JOBS           ?= $(shell nproc 2>/dev/null || echo 4)
PYTHON         ?= python3
SCENARIO       ?= configs/scenarios/nominal.yaml
MC_SCENARIO    ?= configs/scenarios/monte_carlo.yaml
CMAKE_ARGS     ?=

BIN            := $(BUILD_DIR)/bin

.PHONY: all configure build test ctest run run-scenarios trim monte-carlo integrators \
        figures run-all clean clean-results help

all: build

help:
	@sed -n '4,15p' Makefile

configure:
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) $(CMAKE_ARGS)

build: configure
	cmake --build $(BUILD_DIR) -j $(JOBS)

# The Catch2 binary prints a compact summary; `make ctest` runs the same tests through
# CTest, which is what CI uses because it reports per-test-case results.
test: build
	$(BIN)/aether_tests

ctest: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

run: build
	$(BIN)/aether_sim $(SCENARIO)

run-scenarios: build
	$(BIN)/aether_sim configs/scenarios/nominal.yaml
	$(BIN)/aether_sim configs/scenarios/ideal.yaml
	$(BIN)/aether_sim configs/scenarios/nominal_pid.yaml
	$(BIN)/aether_sim configs/scenarios/wind_disturbance.yaml

trim: build
	$(BIN)/aether_trim $(SCENARIO) --output results/linear --sweep 16:34:19

monte-carlo: build
	$(BIN)/aether_mc $(MC_SCENARIO)

integrators: build
	$(BIN)/aether_integrators $(SCENARIO) --output results/integrators

figures:
	$(PYTHON) python/scripts/make_figures.py

run-all: build test trim run-scenarios integrators monte-carlo figures
	@echo
	@echo "All artefacts are under results/ (figures in results/figures/)."

clean:
	rm -rf $(BUILD_DIR)

clean-results:
	rm -rf results/nominal results/nominal_pid results/ideal results/wind \
	       results/monte_carlo/nominal_trial
