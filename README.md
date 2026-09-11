# Drone Planner - C++/WASM engine

C++ engine for the Drone Planner flight-path simulator, compiled to
WebAssembly for the browser. Produces flight trajectories and CoDrone EDU
Python code that match the original JavaScript version exactly.

## Layout

```
src/aero/       flight physics (AeroEngine: thrust, drag, battery, altitude hold)
src/sim/        command interpreter (Simulator), expression evaluator (Eval),
                obstacle collision and boundary logic
src/codegen/    CoDrone EDU Python code generator
src/bridge/     JSON parsing/serialization and C exports for the WASM build
ui/             browser UI (Three.js scene, command palette, telemetry)
```

## Build

Native (for tests):

```
cmake -B build && cmake --build build
ctest --test-dir build
```

WebAssembly (requires the Emscripten SDK):

```
em++ -O2 -std=c++17 --no-entry -sMODULARIZE -sEXPORT_NAME=Engine \
  -sEXPORTED_FUNCTIONS=_engineSimulateC,_engineGenerateCodeC,_malloc,_free \
  -I src src/aero/AeroEngine.cpp src/sim/Eval.cpp src/sim/Simulator.cpp \
  src/sim/Obstacle.cpp src/codegen/CodeGenerator.cpp src/bridge/Json.cpp \
  src/bridge/bridge.cpp -o ui/engine.js
```

The generated `ui/engine.js` and `ui/engine.wasm` are the files the UI loads.

## API

The engine exposes two C entry points to JavaScript (`ccall`/`cwrap`):

- `engineSimulateC(json)` - takes a plan (commands, drones, obstacles,
  boundary) and returns per-drone positions, telemetry, collisions, and
  total duration
- `engineGenerateCodeC(json)` - takes commands and returns CoDrone EDU Python

## License

MIT