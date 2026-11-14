# Monster

A Stunts-style stunt driving game with a monster truck, written in C++ and
compiled to WebAssembly. One truck on a figure-eight circuit of about 990 m
with a row of crushable wrecks, a tunnel, a jump over a sunken dirt road, a
loop, and a bridge with a hole in it.

## The race

Three laps against three computer driven trucks, BLUE, YELLOW and GREEN, from a
staggered grid behind the start line with the player at the back. The HUD shows
the player's position in the field and the lap out of three. The race ends when
the player finishes; R restarts it from the grid.

The opponents drive by the same rules as the player: they clear the same gates
to be credited a lap, they are set upright again after a crash, and one that
stops making progress for six seconds is put back at its last gate. Trucks bump
each other but do not drive through each other. The wrecks' points go to whoever
flattens them.

## The circuit

The two straights of the eight cross at different levels. A lap runs: start
line, tunnel under the bridge, the jump over the dirt road, two right-hand
bends, loop, another
right-hand bend, over the bridge and its hole, two left-hand bends, the last
gate, the row
of wrecks, a last left-hand bend and the run back to the line. The starting grid
stands on that last straight.

The loop is a helix on a slightly turned cylinder, so the road enters and
leaves it tangentially: keep the wheels straight and the throttle open and the
truck comes out in the exit lane. It needs roughly 75 km/h at the entry.
Track points must be passed in order, so a lap only counts if the loop and the
bridge are driven.

The bridge deck is missing for six metres at the top of the climb, well before
the middle of the bridge, and the road kicks up into a lip just before the hole.
At racing speed the truck flies off the lip and lands on the level deck beyond;
too slow, and it drops through to the road underneath. A dirt road runs
alongside the bridge on its right, sunk into a cutting with grass banks in the
darker of the two grass shades; the road out of the tunnel jumps it between two
ramps. Two dirt slip roads branch off the main road at an angle, like the arms of a
junction, before the bridge climbs and after it comes down; they start underneath
the tarmac, so there is no seam, and run into the ends of the cutting,
so the dirt road is a way round the hole. Coming out of its far end counts as
having passed the bridge.

The opponents take the dirt road on three laps out of four, decided afresh
every lap. It is the slower way round, so the bridge is where the player can
overtake.

The going differs by surface. The road and the bridge are quickest, the dirt
road a little slower, and the grass slower still, so cutting a corner costs more
than it saves.

## Controls

| Key | On the ground | In the air |
|---|---|---|
| Up / W | throttle | lifts the nose |
| Down / S | brake, then reverse | drops the nose |
| Left, Right / A, D | steer | |
| Space | set the truck back on the road, upright, keeping the score | |
| R | restart the run | |

A truck left on its roof or side is set back on the road on its own after a few
seconds, so Space only saves waiting. While it is wrecked the controls are
ignored, so holding a key cannot flip it over and launch it.

Touch devices get on-screen buttons.

## Scoring

- Crushing the row of three wrecks: up to 300 points over the row, plus 200 when
  the last of them is flattened.
- Jumps of 0.5 s or longer: 100 points per second in the air plus 10 per metre,
  times 3 for a perfect landing (within 10 degrees of the ground), times 2 for a
  good one (within 20 degrees), times 1 for a rough one. Touching down on the
  chassis or roof is a crash and scores nothing. Each full flip adds 500.
- Driving through the top of the loop: 500 points, once per lap.
- Lap time and best lap time are shown next to the speed.

## Build

    make test                          # native unit tests, any C++17 compiler
    make wasm WASI_SDK=/path/to/sdk    # docs/monster.wasm
    make serve                         # http://localhost:8000
    node tests/smoke.mjs               # drives the wasm build through the course

The WebAssembly build uses the [WASI SDK](https://github.com/WebAssembly/wasi-sdk/releases)
(tested with version 25). Unpack it and pass its path as `WASI_SDK`; the default
is `/opt/wasi-sdk`.

## Hosting on GitHub Pages

The site is the `docs` folder, served straight from the branch.

1. Create a repository and push this folder to its `main` branch.
2. In the repository settings, under Pages, set the source to "Deploy from a
   branch", branch `main`, folder `/docs`.
3. The site appears at `https://<user>.github.io/<repository>/`.

`docs/monster.wasm` is committed, because Pages publishes only what is in the
repository. After changing anything under `src`, run `make wasm` and commit the
rebuilt file along with the sources. The workflow in
`.github/workflows/ci.yml` runs the tests on every push and fails if the
committed `monster.wasm` does not match the sources.

## Layout

| Path | Contents |
|---|---|
| `src/Math.h` | vector, quaternion and matrix types |
| `src/Track.*` | centreline of the circuit: straights, bends, bridge profile, loop |
| `src/Terrain.*` | collision world: ground plane, road surfaces, walls, jump ramps, the row of wrecks; ray casts |
| `src/Truck.*` | rigid body, ray-cast suspension, tyres, air control |
| `src/Judge.*` | air time, flips and landing rating |
| `src/Driver.*` | the computer driver: follows the centreline, brakes for bends, floors it for the loop; its `Style` sets the pace |
| `src/Racer.*` | one competitor: truck, judge, controls or driver, lap, checkpoint and race state, colour, name |
| `src/Game.*` | the race: the list of racers (index 0 is the player), the grid, fixed time step loop, the rules applied per racer, contact between trucks, positions, score, HUD text |
| `src/Color.h` | RGB colour, shared by the game core and the renderer |
| `src/Mesh.*`, `src/Scene.*` | flat shaded triangle list and chase camera |
| `src/wasm_api.cpp` | C functions exported to JavaScript |
| `docs/` | the published site: page, WebGL drawing, input, built `monster.wasm` |
| `tests/` | native tests (physics alone on the circuit, then full races) and the wasm smoke test |

The game core (`Track`, `Terrain`, `Truck`, `Judge`, `Driver`, `Racer`, `Game`) has no platform
dependencies. The C++ side produces a vertex buffer and a view-projection
matrix each frame; `web/monster.js` only uploads and draws them.

Collision is done with ray casts against the road faces, so surfaces can be
stacked (bridge over road) or upside down (loop). Each wheel casts a fan of
rays along the suspension axis; chassis and roof points are tested with rays
from the centre of mass. Trees and mountains are scenery without collision.

Trucks are not in the ray cast world. Each carries a hull of six spheres, and
where the hulls of two trucks overlap they are pushed apart by a spring and a
damper, level with the ground, so a bump can shove or turn a truck but never
lift it or press it into the road.

The wasm build is compiled without exceptions, so contract checks go through
`require()` in `src/Check.h`: it throws `std::logic_error` natively and traps
in the browser.
