// Monster - an RGB colour
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

// Struct Color
struct Color
{
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;

    Color() = default;
    Color(float r, float g, float b)
        :
        r(r)
        , g(g)
        , b(b)
    {
    }

    Color scaled(float s) const { return Color(r*s, g*s, b*s); }
};
