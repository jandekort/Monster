// Monster - a simple driver that follows the centreline, used by the tests
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Game.h"

// Class Autopilot
class Autopilot
{
public:
    Autopilot();

    Controls drive(const Game& game) const;

private:
    double m_lookAhead;    // steering target distance [m]
    double m_steerGain;
    double m_cornerSpeed;  // [m/s]
    double m_braking;      // deceleration assumed when planning for a bend [m/s^2]
    double m_planDistance; // how far ahead bends are considered [m]
};
