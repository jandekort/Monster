// Monster - browser platform layer: loads the WebAssembly game, feeds it input,
// draws the vertex buffer it produces with WebGL and shows the HUD.
// Author: Jan de Kort
// Created: 2026-11-14
"use strict";

(function () {
    const ASPECT = 640 / 400;
    const FLOATS_PER_VERTEX = 6;
    const SKY = [0.45, 0.70, 0.95];

    const canvas = document.getElementById("canvas");
    const frame = document.getElementById("frame");
    const hud = {
        speed: document.getElementById("speed"),
        score: document.getElementById("score"),
        message: document.getElementById("message"),
        error: document.getElementById("error")
    };
    const keys = { gas: false, brake: false, left: false, right: false };

    function fail(text) {
        hud.error.textContent = text;
    }

    // Layout: largest 16:10 frame that fits the window
    function layout() {
        const width = Math.min(window.innerWidth, window.innerHeight * ASPECT);
        frame.style.width = Math.floor(width) + "px";
        frame.style.height = Math.floor(width / ASPECT) + "px";
    }
    window.addEventListener("resize", layout);
    layout();

    // Input
    const keyMap = {
        ArrowUp: "gas", KeyW: "gas",
        ArrowDown: "brake", KeyS: "brake",
        ArrowLeft: "left", KeyA: "left",
        ArrowRight: "right", KeyD: "right"
    };
    let resetRequested = false;
    let recoverRequested = false;
    let setupRequested = 0;

    window.addEventListener("keydown", function (event) {
        if (keyMap[event.code]) {
            keys[keyMap[event.code]] = true;
            event.preventDefault();
        } else if (event.code === "KeyR") {
            resetRequested = true;
        } else if (/^Digit[1-9]$/.test(event.code)) {
            setupRequested = Number(event.code.slice(5));
            event.preventDefault();
        } else if (event.code === "Space" || event.code === "Enter") {
            recoverRequested = true;
            event.preventDefault();
        }
    });
    window.addEventListener("keyup", function (event) {
        if (keyMap[event.code]) {
            keys[keyMap[event.code]] = false;
            event.preventDefault();
        }
    });
    window.addEventListener("blur", function () {
        Object.keys(keys).forEach(function (name) { keys[name] = false; });
    });
    document.querySelectorAll(".pad button").forEach(function (button) {
        const name = button.dataset.key;
        const set = function (down) {
            return function (event) {
                event.preventDefault();
                if (name === "reset") {
                    resetRequested = resetRequested || down;
                } else if (name === "recover") {
                    recoverRequested = recoverRequested || down;
                } else {
                    keys[name] = down;
                }
            };
        };
        button.addEventListener("pointerdown", set(true));
        button.addEventListener("pointerup", set(false));
        button.addEventListener("pointercancel", set(false));
        button.addEventListener("pointerleave", set(false));
    });

    // WebGL
    const gl = canvas.getContext("webgl", { antialias: false, alpha: false });
    if (!gl) {
        fail("WebGL is not available in this browser");
        return;
    }

    function compile(type, source) {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            throw new Error(gl.getShaderInfoLog(shader));
        }
        return shader;
    }

    const program = gl.createProgram();
    gl.attachShader(program, compile(gl.VERTEX_SHADER,
        "attribute vec3 aPosition;" +
        "attribute vec3 aColor;" +
        "uniform mat4 uViewProjection;" +
        "varying vec3 vColor;" +
        "void main() { vColor = aColor; gl_Position = uViewProjection * vec4(aPosition, 1.0); }"));
    gl.attachShader(program, compile(gl.FRAGMENT_SHADER,
        "precision mediump float;" +
        "varying vec3 vColor;" +
        "void main() { gl_FragColor = vec4(vColor, 1.0); }"));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        fail("Shader link failed");
        return;
    }
    gl.useProgram(program);

    const aPosition = gl.getAttribLocation(program, "aPosition");
    const aColor = gl.getAttribLocation(program, "aColor");
    const uViewProjection = gl.getUniformLocation(program, "uViewProjection");
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.enableVertexAttribArray(aPosition);
    gl.enableVertexAttribArray(aColor);
    gl.vertexAttribPointer(aPosition, 3, gl.FLOAT, false, 4 * FLOATS_PER_VERTEX, 0);
    gl.vertexAttribPointer(aColor, 3, gl.FLOAT, false, 4 * FLOATS_PER_VERTEX, 12);
    gl.enable(gl.DEPTH_TEST);
    gl.clearColor(SKY[0], SKY[1], SKY[2], 1.0);

    // The game only needs these WASI calls for stdio, which it never uses
    const wasi = {
        fd_close: function () { return 0; },
        fd_seek: function () { return 0; },
        fd_write: function () { return 0; }
    };

    function readString(game, pointer) {
        const bytes = new Uint8Array(game.memory.buffer);
        let end = pointer;
        while (bytes[end] !== 0) {
            ++end;
        }
        return new TextDecoder().decode(bytes.subarray(pointer, end));
    }

    function start(game) {
        if (game._initialize) {
            game._initialize();
        }
        game.monster_init();
        window.monster = game; // handy in the browser console

        let last = performance.now();
        function tick(now) {
            const elapsed = Math.min(0.1, (now - last) / 1000.0);
            last = now;

            if (resetRequested) {
                resetRequested = false;
                game.monster_reset();
            }
            if (recoverRequested) {
                recoverRequested = false;
                game.monster_recover();
            }
            if (setupRequested > 0) {
                game.monster_select_setup(setupRequested);
                setupRequested = 0;
            }
            game.monster_set_controls(keys.gas ? 1.0 : 0.0, keys.brake ? 1.0 : 0.0,
                (keys.right ? 1.0 : 0.0) - (keys.left ? 1.0 : 0.0));
            game.monster_step(elapsed);

            // Views are recreated every frame because the memory may have grown
            const count = game.monster_render(ASPECT);
            const vertices = new Float32Array(game.memory.buffer, game.monster_vertices(), count * FLOATS_PER_VERTEX);
            const matrix = new Float32Array(game.memory.buffer, game.monster_view_projection(), 16);

            gl.viewport(0, 0, canvas.width, canvas.height);
            gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
            gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.DYNAMIC_DRAW);
            gl.uniformMatrix4fv(uViewProjection, false, matrix);
            gl.drawArrays(gl.TRIANGLES, 0, count);

            const fields = readString(game, game.monster_hud()).split("|");
            hud.speed.innerHTML = (Number(fields[14]) > 1 ? "P" + fields[13] + "/" + fields[14] + "&nbsp; " : "") +
                fields[0] + " km/h<br>LAP " + fields[4] + "/" + fields[15] + "&nbsp; " + fields[5] + " s" +
                (Number(fields[6]) > 0.0 ? "<br>BEST LAP " + fields[6] + " s" : "") +
                "<br>CHECKPOINT " + fields[7] + "/" + fields[8] +
                "<br>TYRES " + fields[9] + " " + fields[10] +
                "<br>grip " + fields[11] + " / top " + fields[12] + " km/h";
            hud.score.innerHTML = "SCORE " + fields[1] + "<br>BEST " + fields[2];
            hud.message.textContent = fields[3] || "";

            window.monsterFrames = (window.monsterFrames || 0) + 1;
            requestAnimationFrame(tick);
        }
        requestAnimationFrame(tick);
    }

    fetch("monster.wasm")
        .then(function (response) {
            if (!response.ok) {
                throw new Error("monster.wasm: HTTP " + response.status);
            }
            return response.arrayBuffer();
        })
        .then(function (bytes) {
            return WebAssembly.instantiate(bytes, { wasi_snapshot_preview1: wasi });
        })
        .then(function (result) {
            start(result.instance.exports);
        })
        .catch(function (error) {
            fail("Could not start the game: " + error.message);
        });
})();
