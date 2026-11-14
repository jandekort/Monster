// Monster - smoke test of the WebAssembly build: drives the whole course in Node
// Usage: node tests/smoke.mjs [path to monster.wasm]
import { readFileSync } from "node:fs";

const path = process.argv[2] || "docs/monster.wasm";
const wasi = { fd_close: () => 0, fd_seek: () => 0, fd_write: () => 0 };
const { instance } = await WebAssembly.instantiate(readFileSync(path), { wasi_snapshot_preview1: wasi });
const game = instance.exports;

function hud() {
    const bytes = new Uint8Array(game.memory.buffer);
    let end = game.monster_hud();
    const begin = end;
    while (bytes[end] !== 0) {
        ++end;
    }
    return new TextDecoder().decode(bytes.subarray(begin, end)).split("|");
}

let failures = 0;
function check(condition, what) {
    console.log((condition ? "  ok    " : "  FAIL  ") + what);
    failures += condition ? 0 : 1;
}

game._initialize();
game.monster_init();

// Alone on the circuit first: no opponents to crush the wreck or get in the way
game.monster_start_race(0);
game.monster_set_controls(1.0, 0.0, 0.0);
let sawAir = false;
let sawLanding = false;
for (let frame = 0; frame < 60 * 15; ++frame) {
    game.monster_step(1.0 / 60.0);
    const message = hud()[3];
    sawAir = sawAir || message.startsWith("AIR");
    sawLanding = sawLanding || message.includes("LANDING");
}
const count = game.monster_render(1.6);
const vertices = new Float32Array(game.memory.buffer, game.monster_vertices(), count * 6);
const matrix = new Float32Array(game.memory.buffer, game.monster_view_projection(), 16);
const fields = hud();
console.log("    hud " + fields.join(" | ") + ", " + count + " vertices");

check(count > 1000 && count % 3 === 0, "scene has triangles");
check(vertices.every(Number.isFinite), "vertices are finite");
check(matrix.every(Number.isFinite), "view projection is finite");
check(Number(fields[1]) >= 300, "the jump scores points");
check(fields.length === 17 && fields[4] === "1" && Number(fields[5]) > 10.0, "lap and lap time are reported");
check(Number(fields[7]) >= 1 && fields[8] === "4", "a gate driven through is counted");
check(sawAir, "truck gets airborne");
check(sawLanding, "landing is rated");

const beforeRecovery = Number(fields[1]);
game.monster_recover();
game.monster_step(1.0 / 60.0);
check(Number(hud()[1]) === beforeRecovery, "recovery keeps the score");

check(hud()[9] === "3" && hud()[10] === "STANDARD", "the standard tyre setup is the default");
game.monster_select_setup(1);
game.monster_step(1.0 / 60.0);
check(hud()[9] === "1" && Number(hud()[11]) > 1.1, "a softer setup can be selected");
game.monster_select_setup(99);
game.monster_step(1.0 / 60.0);
check(hud()[9] === String(5), "an out of range setup is clamped");

game.monster_reset();
check(hud()[9] === "5", "reset keeps the tyre setup");
check(hud()[1] === "0" && Number(hud()[2]) >= 300, "reset keeps the best score");

// The race: three opponents on the grid ahead of the player
check(hud()[13] === "1" && hud()[14] === "1", "alone, the player is first of one");
game.monster_start_race(3);
game.monster_step(1.0 / 60.0);
const soloCount = count;
const raceCount = game.monster_render(1.6);
check(hud()[13] === "4" && hud()[14] === "4" && hud()[15] === "3", "the race starts with the player fourth of four, over three laps");
check(raceCount > soloCount, "the opponents are drawn (" + raceCount + " vertices against " + soloCount + ")");
game.monster_set_controls(1.0, 0.0, 0.0);
let frameTime = 0.0;
for (let frame = 0; frame < 60 * 30; ++frame) {
    const before = performance.now();
    game.monster_step(1.0 / 60.0);
    game.monster_render(1.6);
    frameTime += performance.now() - before;
}
const raceVertices = new Float32Array(game.memory.buffer, game.monster_vertices(), game.monster_render(1.6) * 6);
console.log("    hud " + hud().join(" | "));
console.log("    step and scene build: " + (frameTime / (60 * 30)).toFixed(3) + " ms per frame");
check(raceVertices.every(Number.isFinite), "the race renders finite vertices");
check(Number(hud()[13]) >= 1 && Number(hud()[13]) <= 4 && hud()[16] === "0", "the player has a position in the field");
check(frameTime / (60 * 30) < 8.0, "a frame of simulation and scene building fits well inside 16.7 ms");
game.monster_reset();
check(hud()[13] === "4" && hud()[4] === "1" && hud()[1] === "0", "restarting puts the player back on the grid");

process.exit(failures === 0 ? 0 : 1);
