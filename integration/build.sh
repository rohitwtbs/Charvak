#!/bin/bash

emcc main.cpp -o index.html \
    -I/raylib/src \
    /raylib/build/raylib/libraylib.a \
    -s USE_GLFW=3 \
    -s ASYNCIFY \
    -s WASM=1 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s USE_WEBGL2=1 \
    -s FULL_ES3=1 \
    -std=c++17
