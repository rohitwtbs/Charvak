FROM emscripten/emsdk:latest

# Install dependencies
RUN apt-get update && apt-get install -y git make wget

# Install the latest version of CMake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.25.0/cmake-3.25.0-linux-x86_64.sh && \
    chmod +x cmake-3.25.0-linux-x86_64.sh && \
    ./cmake-3.25.0-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.25.0-linux-x86_64.sh

# Clone raylib and build it for WebAssembly
RUN git clone https://github.com/raysan5/raylib.git /raylib && \
    mkdir /raylib/build && \
    cd /raylib/build && \
    emcmake cmake -DBUILD_EXAMPLES=OFF -DPLATFORM=Web .. && \
    emmake make

# Set the working directory for the application
WORKDIR /app

# Copy application files
COPY main.cpp .
COPY build.sh .

# Make the build script executable
RUN chmod +x build.sh

# Default command to build the application
CMD ["./build.sh"]