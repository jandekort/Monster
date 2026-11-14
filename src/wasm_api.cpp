// Monster - C interface exported to JavaScript
// Author: Jan de Kort
// Created: 2026-11-14
#include "Game.h"
#include "Scene.h"

#include <string>

#define MONSTER_EXPORT(name) __attribute__((export_name(#name)))

namespace
{
    // Struct App: everything the page talks to
    struct App
    {
        App()
            :
            m_game(Game::maxOpponents())
            , m_camera()
            , m_scene()
            , m_viewProjection()
            , m_hud()
        {
        }

        Game m_game;
        ChaseCamera m_camera;
        Scene m_scene;
        Mat4 m_viewProjection;
        std::string m_hud;
    };

    App& app()
    {
        static App instance;
        return instance;
    }
}

extern "C"
{

MONSTER_EXPORT(monster_init) void monster_init()
{
    app().m_game.reset();
    app().m_camera.snap();
}

MONSTER_EXPORT(monster_reset) void monster_reset()
{
    app().m_game.reset();
    app().m_camera.snap();
}

// A fresh race against the given number of opponents; none leaves the circuit to the player
MONSTER_EXPORT(monster_start_race) void monster_start_race(int opponents)
{
    const int most = (int) Game::maxOpponents();
    app().m_game = Game((size_t) (opponents < 0 ? 0 : (opponents > most ? most : opponents)));
    app().m_camera.snap();
}

// Tyre setup, 1 is the softest. Out of range values are clamped.
MONSTER_EXPORT(monster_select_setup) void monster_select_setup(int index)
{
    app().m_game.selectSetup(index);
}

// Sets the truck upright on the road where it last was, keeping the score
MONSTER_EXPORT(monster_recover) void monster_recover()
{
    app().m_game.recover();
}

// steer: -1->left, +1->right
MONSTER_EXPORT(monster_set_controls) void monster_set_controls(float throttle, float brake, float steer)
{
    Controls controls;
    controls.m_throttle = throttle;
    controls.m_brake = brake;
    controls.m_steer = steer;
    app().m_game.setControls(controls);
}

MONSTER_EXPORT(monster_step) void monster_step(float elapsed)
{
    App& a = app();
    a.m_game.step(elapsed);
    a.m_camera.update(a.m_game.getTruck(), elapsed > 0.0f ? elapsed : 0.0);
}

// Builds the frame; returns the number of vertices
MONSTER_EXPORT(monster_render) int monster_render(float aspect)
{
    App& a = app();
    a.m_scene.build(a.m_game, a.m_camera.getEye());
    a.m_viewProjection = a.m_camera.viewProjection(aspect);
    return (int) a.m_scene.getMesh().vertexCount();
}

// Six floats per vertex: x, y, z, r, g, b
MONSTER_EXPORT(monster_vertices) const float* monster_vertices()
{
    return app().m_scene.getMesh().data();
}

// Sixteen floats, column-major
MONSTER_EXPORT(monster_view_projection) const float* monster_view_projection()
{
    return app().m_viewProjection.m;
}

// Zero terminated "speed|score|best|message|lap|lap time|best lap time"
MONSTER_EXPORT(monster_hud) const char* monster_hud()
{
    app().m_hud = app().m_game.hudString();
    return app().m_hud.c_str();
}

}
