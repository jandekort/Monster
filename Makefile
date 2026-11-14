# Monster - build file
#   make test   native unit tests (any C++17 compiler)
#   make wasm   web/monster.wasm (needs the WASI SDK, see README.md)
#   make serve  local web server on http://localhost:8000

WASI_SDK ?= /opt/wasi-sdk
CXX      ?= c++

CORE := src/Track.cpp src/Terrain.cpp src/Truck.cpp src/Judge.cpp src/Game.cpp
VIEW := src/Mesh.cpp src/Scene.cpp
HEADERS := $(wildcard src/*.h)

WASM_FLAGS := --target=wasm32-wasi -std=c++17 -O2 -Wall -Wextra \
	-fno-exceptions -fno-rtti -mexec-model=reactor \
	-Wl,--strip-all -Wl,--no-entry

.PHONY: all wasm test serve clean

all: test wasm

wasm: web/monster.wasm

web/monster.wasm: $(CORE) $(VIEW) src/wasm_api.cpp $(HEADERS)
	$(WASI_SDK)/bin/clang++ $(WASM_FLAGS) -Isrc $(CORE) $(VIEW) src/wasm_api.cpp -o $@

build/test_game: $(CORE) tests/Autopilot.cpp tests/Autopilot.h tests/test_game.cpp $(HEADERS)
	@mkdir -p build
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Isrc -Itests $(CORE) tests/Autopilot.cpp tests/test_game.cpp -o $@

test: build/test_game
	./build/test_game

serve: wasm
	python3 -m http.server 8000 --directory web

clean:
	rm -rf build web/monster.wasm
