# Monster

A Stunts-style stunt driving game with a monster truck, written in C++ and
compiled to WebAssembly. One truck on a figure-eight circuit of about 990 m
with a crushable wreck, a tunnel, a jump, a loop and a bridge.

## The circuit

The two straights of the eight cross at different levels. A lap runs: start
line, wreck, tunnel under the bridge, jump, two right-hand bends, loop, another
right-hand bend, over the bridge, three left-hand bends back to the start.

The loop is a helix on a slightly turned cylinder, so the road enters and
leaves it tangentially: keep the wheels straight and the throttle open and the
truck comes out in the exit lane. It needs roughly 75 km/h at the entry.
Track points must be passed in order, so a lap only counts if the loop and the
bridge are driven.

## Controls

| Key | On the ground | In the air |
|---|---|---|
| Up / W | throttle | lifts the nose |
| Down / S | brake, then reverse | drops the nose |
| Left, Right / A, D | steer | |
| R | restart the run | |

Touch devices get on-screen buttons.

## Scoring

- Crushing the wreck: up to 300 points, plus 200 when it is completely flat.
- Jumps of 0.5 s or longer: 100 points per second in the air plus 10 per metre,
  times 3 for a perfect landing (within 10 degrees of the ground), times 2 for a
  good one (within 20 degrees), times 1 for a rough one. Touching down on the
  chassis or roof is a crash and scores nothing. Each full flip adds 500.
- Driving through the top of the loop: 500 points, once per lap.
- Lap time and best lap time are shown next to the speed.

## Build

    make test                          # native unit tests, any C++17 compiler
    make wasm WASI_SDK=/path/to/sdk    # web/monster.wasm
    make serve                         # http://localhost:8000
    node tests/smoke.mjs               # drives the wasm build through the course

The WebAssembly build uses the [WASI SDK](https://github.com/WebAssembly/wasi-sdk/releases)
(tested with version 25). Unpack it and pass its path as `WASI_SDK`; the default
is `/opt/wasi-sdk`.

## Hosting on GitHub Pages

1. Create a repository and push this folder to its `main` branch.
2. In the repository settings, under Pages, set the source to "GitHub Actions".
3. The workflow in `.github/workflows/pages.yml` runs the tests, builds
   `monster.wasm` and publishes the `web` folder on every push to `main`.

## Layout

| Path | Contents |
|---|---|
| `src/Math.h` | vector, quaternion and matrix types |
| `src/Track.*` | centreline of the circuit: straights, bends, bridge profile, loop |
| `src/Terrain.*` | collision world: ground plane, road surfaces, walls, jump ramps, wreck; ray casts |
| `src/Truck.*` | rigid body, ray-cast suspension, tyres, air control |
| `src/Judge.*` | air time, flips and landing rating |
| `src/Game.*` | fixed time step loop, score, laps, HUD text |
| `src/Mesh.*`, `src/Scene.*` | flat shaded triangle list and chase camera |
| `src/wasm_api.cpp` | C functions exported to JavaScript |
| `web/` | page, WebGL drawing, input |
| `tests/` | native tests (with an autopilot that drives a full lap) and the wasm smoke test |

The game core (`Track`, `Terrain`, `Truck`, `Judge`, `Game`) has no platform
dependencies. The C++ side produces a vertex buffer and a view-projection
matrix each frame; `web/monster.js` only uploads and draws them.

Collision is done with ray casts against the road faces, so surfaces can be
stacked (bridge over road) or upside down (loop). Each wheel casts a fan of
rays along the suspension axis; chassis and roof points are tested with rays
from the centre of mass. Trees and mountains are scenery without collision.

The wasm build is compiled without exceptions, so contract checks go through
`require()` in `src/Check.h`: it throws `std::logic_error` natively and traps
in the browser.
