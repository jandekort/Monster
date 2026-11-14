// Monster - a computer driver: follows the centreline, brakes for bends, floors it for the loop
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Truck.h"

class Track;

// Class Driver
class Driver
{
public:
    // Struct Style: how a driver takes the circuit
    struct Style
    {
        double m_lookAhead = 14.0;    // steering target distance [m]
        double m_steerGain = 2.5;
        double m_cornerSpeed = 12.0;  // [m/s]
        double m_braking = 4.5;       // deceleration assumed when planning for a bend [m/s^2]
        double m_planDistance = 80.0; // how far ahead bends and the loop are considered [m]
        double m_throttleCap = 1.0;   // most throttle used, except on the run into the loop
        double m_detourPace = 1.3;    // speed kept on the dirt road, as a multiple of m_cornerSpeed
    };

    Driver();
    explicit Driver(const Style& style);

    // progress is the index of the last track point the truck reached
    Controls drive(const Truck& truck, size_t progress, const Track& track) const;

    // Along a route that is not the circuit itself, such as the dirt road round the bridge
    Controls follow(const Truck& truck, const std::vector<Vec3>& path) const;

    // Accessor
    const Style& getStyle() const { return m_style; }

private:
    Style m_style;
};
