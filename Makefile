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
#
#   make dashboard     build the frontend and serve the whole dashboard on :8000
#   make dashboard-dev run the API and the Vite dev server together (hot reload)
#   make dashboard-test run the FastAPI service's test suite
#
#   make plate         render the shareable animation and stills (see media/plate)
#
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

FRONTEND_DIR   := web/frontend
BACKEND_DIR    := web/backend
DASHBOARD_PORT ?= 8000

.PHONY: all configure build test ctest run run-scenarios trim monte-carlo integrators \
        figures run-all clean clean-results help \
        dashboard dashboard-build dashboard-dev dashboard-test plate

all: build

help:
	@sed -n '4,20p' Makefile

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

# ---------------------------------------------------------------------------------------
# Web dashboard. The service runs the compiled binaries, so `build` is a real prerequisite.
# ---------------------------------------------------------------------------------------
$(FRONTEND_DIR)/node_modules: $(FRONTEND_DIR)/package-lock.json
	cd $(FRONTEND_DIR) && npm ci --no-audit --no-fund
	@touch $@

dashboard-build: $(FRONTEND_DIR)/node_modules
	cd $(FRONTEND_DIR) && npm run build

dashboard: build dashboard-build
	PORT=$(DASHBOARD_PORT) PYTHONPATH=$(BACKEND_DIR) \
	  $(PYTHON) -m uvicorn app.main:app --host 0.0.0.0 --port $(DASHBOARD_PORT)

# Two processes: uvicorn with reload on :8000 and Vite on :5173, which proxies /api to it.
dashboard-dev: build $(FRONTEND_DIR)/node_modules
	PYTHONPATH=$(BACKEND_DIR) $(PYTHON) -m uvicorn app.main:app --reload --port 8000 & \
	  cd $(FRONTEND_DIR) && npm run dev; kill %1

# The shareable render. Slow: it captures 660 frames through a software renderer.
plate: build $(FRONTEND_DIR)/node_modules
	media/plate/build.sh

dashboard-test: build
	cd $(BACKEND_DIR) && $(PYTHON) -m pytest -q

clean:
	rm -rf $(BUILD_DIR) $(FRONTEND_DIR)/dist

clean-results:
	rm -rf results/nominal results/nominal_pid results/ideal results/wind \
	       results/monte_carlo/nominal_trial
