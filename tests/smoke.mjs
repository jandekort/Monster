// Monster - smoke test of the WebAssembly build: drives the whole course in Node
// Usage: node tests/smoke.mjs [path to monster.wasm]
import { readFileSync } from "node:fs";

const path = process.argv[2] || "web/monster.wasm";
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
game.monster_set_controls(1.0, 0.0, 0.0);
let sawAir = false;
let sawLanding = false;
for (let frame = 0; frame < 60 * 11; ++frame) {
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
check(Number(fields[1]) >= 500, "crushing and jumping score points");
check(fields.length === 7 && fields[4] === "1" && Number(fields[5]) > 10.0, "lap and lap time are reported");
check(sawAir, "truck gets airborne");
check(sawLanding, "landing is rated");

game.monster_reset();
check(hud()[1] === "0" && Number(hud()[2]) >= 500, "reset keeps the best score");

process.exit(failures === 0 ? 0 : 1);
