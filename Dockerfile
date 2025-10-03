# Minimal Debian image with cross-compilers
FROM debian:bookworm

# Install cross toolchain and build tools
RUN dpkg --add-architecture arm64 && apt-get update && \
    apt-get install -y --no-install-recommends \
      crossbuild-essential-arm64 cmake ninja-build git pkg-config \
      ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Build context lives here
WORKDIR /src
COPY . /src

# If the project already has CMakeLists.txt at the top, this will just work.
# If not, replace "-S ." with the path to the specific subfolder you want to build,
# e.g. "-S lora_shooting-userside".
# EXAMPLE: build the userside LoRa app
RUN cmake -B build -S lora_shooting-userside -G Ninja \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
 && cmake --build build -j
