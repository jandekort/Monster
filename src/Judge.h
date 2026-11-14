// Monster - scores jumps: air time, distance, flips and landing quality
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Math.h"

class Truck;

// Class Judge
class Judge
{
public:
    enum class Rating
    {
        None,
        Perfect,
        Good,
        Rough,
        Crash
    };

    Judge();

    // Mutator
    void reset();
    void settle(); // forget the current flight, keep the score
    void update(const Truck& truck, double dt);

    // Accessor
    int getScore() const { return m_score; }
    bool isAirborne() const { return m_airborne; }
    double getAirTime() const { return m_airTime; }
    Rating getLastRating() const { return m_lastRating; }
    int getLastPoints() const { return m_lastPoints; }
    int getLastFlips() const { return m_lastFlips; }
    double getMessageClock() const { return m_messageClock; } // seconds since the last verdict

    static const char* ratingName(Rating rating);

private:
    void touchDown(const Truck& truck);
    void verdict();

    // Tuning
    double m_airborneDelay;    // contact loss shorter than this is a bounce [s]
    double m_minAirTime;       // shorter flights are not scored [s]
    double m_settleTime;       // observation window after touchdown [s]
    double m_perfectAlignment; // cosine between truck up and ground normal
    double m_goodAlignment;
    double m_pointsPerSecond;
    double m_pointsPerMeter;
    int m_pointsPerFlip;

    // State
    int m_score;
    bool m_airborne;
    double m_looseClock;  // time without any contact
    double m_airTime;
    Vec3 m_takeoff;
    double m_pitchTravel; // accumulated pitch rotation in the air [rad]
    bool m_settling;      // true->touched down, waiting for the verdict
    double m_settleClock;
    double m_flightTime;
    double m_flightDistance;
    double m_alignment;
    bool m_crashed;
    Rating m_lastRating;
    int m_lastPoints;
    int m_lastFlips;
    double m_messageClock;
};
