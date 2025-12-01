FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ---- Sistema base + ferramentas de build ----
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    curl \
    # ---- C++ deps ----
    libopencv-dev \
    libmosquitto-dev \
    libmosquittopp-dev \
    # ---- Python + GUI ----
    python3 \
    python3-pip \
    python3-tk \
    # ---- Utils ----
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# ---- Bibliotecas Python ----
RUN pip3 install --no-cache-dir \
    matplotlib \
    paho-mqtt

WORKDIR /opt/hovercraft_prj

COPY . .

# Se usar CMake (recomendado se você criar um CMakeLists.txt):
# RUN mkdir -p build && cd build && cmake .. && make -j$(nproc)

# Ou, se for um único arquivo principal (ajuste caminhos!):
# RUN g++ -std=c++17 -O2 -pthread \
#     -o hovercraft_pipeline \
#     src/hovercraft_pipeline.cpp \
#     `pkg-config --cflags --libs opencv4` \
#     -lmosquittopp -lmosquitto

CMD ["/bin/bash"]
