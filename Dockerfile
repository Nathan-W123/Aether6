# syntax=docker/dockerfile:1
#
# Aether-6 dashboard: one image containing the compiled simulator, the built frontend and the
# FastAPI service that connects them. Railway runs this as a single service.
#
#   stage 1  build the C++ binaries
#   stage 2  build the React frontend
#   stage 3  runtime: Python + FastAPI + the artefacts of stages 1 and 2
#
# Stages 1 and 3 share the same Debian release so the C++ binaries find the glibc, libstdc++
# and yaml-cpp they were linked against.

# ---------------------------------------------------------------------------------------
# Stage 1 — simulator
# ---------------------------------------------------------------------------------------
FROM debian:bookworm-slim AS simulator

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      libeigen3-dev \
      libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY CMakeLists.txt ./
COPY include/ include/
COPY src/ src/
COPY apps/ apps/

# Tests are built and run by CI, not by the image: the runtime only needs the binaries.
RUN cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DAETHER_BUILD_TESTS=OFF \
      -DAETHER_ENABLE_WERROR=OFF \
 && cmake --build build --parallel "$(nproc)" \
 && build/bin/aether_sim --help > /dev/null \
 && build/bin/aether_mc --help > /dev/null

# ---------------------------------------------------------------------------------------
# Stage 2 — frontend
# ---------------------------------------------------------------------------------------
FROM node:22-bookworm-slim AS frontend

WORKDIR /web
COPY web/frontend/package.json web/frontend/package-lock.json ./
RUN npm ci --no-audit --no-fund

COPY web/frontend/ ./
RUN npm run build

# ---------------------------------------------------------------------------------------
# Stage 3 — runtime
# ---------------------------------------------------------------------------------------
FROM python:3.11-slim-bookworm AS runtime

# libyaml-cpp-dev is installed rather than the versioned runtime package so the image does
# not hard-code a yaml-cpp soname; apt resolves the matching shared library as its dependency.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    AETHER_ROOT=/app \
    AETHER_BIN_DIR=/app/bin \
    AETHER_RESULTS_DIR=/app/results \
    AETHER_AIRCRAFT=/app/configs/aircraft/aether6_uav.yaml \
    AETHER_STATIC_DIR=/app/static \
    PORT=8000

WORKDIR /app

COPY web/backend/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY --from=simulator /src/build/bin/aether_sim /src/build/bin/aether_mc /app/bin/
COPY --from=frontend  /web/dist/                                          /app/static/
COPY web/backend/app/            /app/app/
COPY configs/aircraft/           /app/configs/aircraft/
COPY results/monte_carlo/summary.json results/monte_carlo/trials.csv \
     results/monte_carlo/envelope.csv                                     /app/results/monte_carlo/
COPY results/linear/                                                      /app/results/linear/

# Run unprivileged; /tmp must stay writable because every run gets its own directory there.
RUN useradd --system --create-home --uid 10001 aether \
 && chown -R aether:aether /app
USER aether

EXPOSE 8000

# Railway's own health check uses railway.json; this one covers plain `docker run`.
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
  CMD python -c "import os,urllib.request,sys; \
sys.exit(0 if urllib.request.urlopen(f\"http://127.0.0.1:{os.environ.get('PORT','8000')}/health\", timeout=4).status == 200 else 1)"

# app.main.main() binds 0.0.0.0 on $PORT, which Railway injects at start.
CMD ["python", "-m", "app.main"]
