FROM debian:bookworm

ARG SRC_DIR="."

RUN dpkg --add-architecture arm64 && apt-get update && \
    apt-get install -y --no-install-recommends \
      crossbuild-essential-arm64 cmake ninja-build git pkg-config \
      ca-certificates && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . /src

# Build the chosen subproject for ARM64
RUN bash -lc 'set -eux; \
    echo "Requested SRC_DIR: ${SRC_DIR}"; \
    test -f "${SRC_DIR}/CMakeLists.txt" || { echo "No CMakeLists.txt in ${SRC_DIR}"; exit 1; }; \
    cmake -B build -S "${SRC_DIR}" -G Ninja \
      -DCMAKE_SYSTEM_NAME=Linux \
      -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
      -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
      -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++; \
    cmake --build build -j'
