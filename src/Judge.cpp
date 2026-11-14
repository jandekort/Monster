// Monster - scores jumps: air time, distance, flips and landing quality
// Author: Jan de Kort
// Created: 2026-11-14
#include "Judge.h"

#include "Truck.h"

#include <cmath>
#include "Check.h"

Judge::Judge()
    :
    m_airborneDelay(0.15)
    , m_minAirTime(0.5)
    , m_settleTime(0.8)
    , m_perfectAlignment(0.985) // within 10 degrees
    , m_goodAlignment(0.94)     // within 20 degrees
    , m_pointsPerSecond(100.0)
    , m_pointsPerMeter(10.0)
    , m_pointsPerFlip(500)
    , m_score(0)
    , m_airborne(false)
    , m_looseClock(0.0)
    , m_airTime(0.0)
    , m_takeoff()
    , m_pitchTravel(0.0)
    , m_settling(false)
    , m_settleClock(0.0)
    , m_flightTime(0.0)
    , m_flightDistance(0.0)
    , m_alignment(1.0)
    , m_crashed(false)
    , m_lastRating(Rating::None)
    , m_lastPoints(0)
    , m_lastFlips(0)
    , m_messageClock(0.0)
{
}

void Judge::reset()
{
    *this = Judge();
}

const char* Judge::ratingName(Rating rating)
{
    switch (rating)
    {
    case Rating::None:
        return "";
    case Rating::Perfect:
        return "PERFECT LANDING";
    case Rating::Good:
        return "GOOD LANDING";
    case Rating::Rough:
        return "ROUGH LANDING";
    case Rating::Crash:
        return "CRASH LANDING";
    default:
        require(false, "Judge::ratingName: unknown rating");
        return "";
    }
}

void Judge::update(const Truck& truck, double dt)
{
    const bool contact = truck.getWheelsOnGround() > 0 || truck.hasBodyContact();
    m_messageClock += dt;

    if (!contact)
    {
        if (m_looseClock == 0.0)
        {
            m_takeoff = truck.getPosition();
            m_pitchTravel = 0.0;
        }
        m_looseClock += dt;
        m_pitchTravel += dot(truck.getAngularVelocity(), truck.right()) * dt;
        if (m_looseClock > m_airborneDelay)
        {
            m_airborne = true;
            m_airTime = m_looseClock;
        }
    }
    else
    {
        if (m_airborne && m_looseClock >= m_minAirTime)
        {
            touchDown(truck);
        }
        m_airborne = false;
        m_looseClock = 0.0;
    }

    if (m_settling)
    {
        m_settleClock += dt;
        if (truck.hasBodyContact() || truck.up().y < 0.4)
        {
            m_crashed = true;
        }
        if (m_settleClock >= m_settleTime)
        {
            verdict();
        }
    }
}

void Judge::touchDown(const Truck& truck)
{
    const Vec3& position = truck.getPosition();
    const Vec3 travelled = position - m_takeoff;
    m_settling = true;
    m_settleClock = 0.0;
    m_flightTime = m_looseClock;
    m_flightDistance = std::sqrt(travelled.x*travelled.x + travelled.z*travelled.z);
    m_alignment = truck.getWheelsOnGround() > 0 ? dot(truck.up(), truck.getContactNormal()) : 0.0;
    m_crashed = truck.hasBodyContact();
    m_lastFlips = (int) std::floor(std::fabs(m_pitchTravel) / (2.0 * M_PI) + 0.15);
}

void Judge::verdict()
{
    m_settling = false;
    m_messageClock = 0.0;

    double multiplier = 1.0;
    if (m_crashed)
    {
        m_lastRating = Rating::Crash;
        multiplier = 0.0;
    }
    else if (m_alignment >= m_perfectAlignment)
    {
        m_lastRating = Rating::Perfect;
        multiplier = 3.0;
    }
    else if (m_alignment >= m_goodAlignment)
    {
        m_lastRating = Rating::Good;
        multiplier = 2.0;
    }
    else
    {
        m_lastRating = Rating::Rough;
    }

    const double base = m_pointsPerSecond*m_flightTime + m_pointsPerMeter*m_flightDistance;
    m_lastPoints = m_crashed ? 0 : (int) std::lround(base * multiplier) + m_pointsPerFlip*m_lastFlips;
    m_score += m_lastPoints;
}
