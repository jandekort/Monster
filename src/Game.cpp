// Monster - the game: one truck, one course, one score
// Author: Jan de Kort
// Created: 2026-11-14
#include "Game.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

Game::Game()
    :
    m_truck()
    , m_terrain()
    , m_judge()
    , m_controls()
    , m_fixedStep(1.0 / 240.0)
    , m_maxElapsed(0.1)
    , m_accumulator(0.0)
    , m_wreckedDelay(2.0)
    , m_messageDuration(3.0)
    , m_crushPoints(300)
    , m_crushBonus(200)
    , m_loopPoints(500)
    , m_gateRadius(12.0)
    , m_gateWindow(10)
    , m_tippedClock(0.0)
    , m_wrecked(false)
    , m_crushClock(-1.0)
    , m_best(0)
    , m_progress(0)
    , m_lap(1)
    , m_lapTime(0.0)
    , m_bestLapTime(-1.0)
    , m_lapClock(-1.0)
    , m_lastLapTime(0.0)
    , m_loopDone(false)
    , m_loopClock(-1.0)
    , m_stuntScore(0)
{
    reset();
}

void Game::reset()
{
    m_best = std::max(m_best, getScore());
    m_terrain.reset();
    m_judge.reset();
    m_truck.reset(Vec3(0.0, m_truck.getRideHeight(), 0.0));
    m_accumulator = 0.0;
    m_tippedClock = 0.0;
    m_wrecked = false;
    m_crushClock = -1.0;
    m_lap = 1;
    m_lapTime = 0.0;
    m_lapClock = -1.0;
    m_loopDone = false;
    m_loopClock = -1.0;
    m_stuntScore = 0;

    { // The truck starts a little past the start line: find the track point it stands on
        const std::vector<PathPoint>& points = m_terrain.getTrack().getPoints();
        m_progress = 0;
        while (m_progress + 1 < points.size() && points[m_progress + 1].m_position.x <= m_truck.getPosition().x)
        {
            ++m_progress;
        }
    }
}

void Game::step(double elapsed)
{
    m_accumulator += clamp(elapsed, 0.0, m_maxElapsed);
    while (m_accumulator >= m_fixedStep)
    {
        m_accumulator -= m_fixedStep;

        m_truck.step(m_fixedStep, m_controls, m_terrain);
        m_judge.update(m_truck, m_fixedStep);
        updateProgress();
        updateStunts();

        { // Wrecked when lying on its side or roof for a while
            const bool tipped = m_truck.up().y < 0.3 && m_truck.getSpeed() < 2.0;
            m_tippedClock = tipped ? m_tippedClock + m_fixedStep : 0.0;
            m_wrecked = m_tippedClock > m_wreckedDelay;
        }

        { // Crush message
            if (m_crushClock >= 0.0)
            {
                m_crushClock += m_fixedStep;
            }
            else if (m_terrain.getWreck().getCrush() >= 1.0)
            {
                m_crushClock = 0.0;
            }
        }
    }
    m_best = std::max(m_best, getScore());
}

// Track points must be reached in order, so the loop and the bridge cannot be skipped
void Game::updateProgress()
{
    const std::vector<PathPoint>& points = m_terrain.getTrack().getPoints();
    const size_t last = points.size() - 1; // coincides with the first point
    m_lapTime += m_fixedStep;
    if (m_lapClock >= 0.0)
    {
        m_lapClock += m_fixedStep;
    }

    for (size_t ahead = m_gateWindow; ahead >= 1; --ahead)
    {
        const size_t index = std::min(m_progress + ahead, last);
        if (index > m_progress && length(points[index].m_position - m_truck.getPosition()) < m_gateRadius)
        {
            m_progress = index;
            break;
        }
    }

    if (m_progress == last)
    {
        m_progress = 0;
        m_lastLapTime = m_lapTime;
        m_bestLapTime = m_bestLapTime < 0.0 ? m_lapTime : std::min(m_bestLapTime, m_lapTime);
        m_lapTime = 0.0;
        m_lapClock = 0.0;
        m_loopDone = false;
        ++m_lap;
    }
}

void Game::updateStunts()
{
    if (m_loopClock >= 0.0)
    {
        m_loopClock += m_fixedStep;
    }

    // Upside down with the wheels on the road: that is the top of the loop
    const bool inverted = m_truck.up().y < -0.7 && m_truck.getWheelsOnGround() >= 2;
    if (inverted && !m_loopDone && m_terrain.getTrack().getPoints()[m_progress].m_kind == PathPoint::Kind::Loop)
    {
        m_loopDone = true;
        m_loopClock = 0.0;
        m_stuntScore += m_loopPoints;
    }
}

int Game::getCrushPoints() const
{
    const double crush = m_terrain.getWreck().getCrush();
    return (int) std::lround(m_crushPoints * crush) + (crush >= 1.0 ? m_crushBonus : 0);
}

int Game::getScore() const
{
    return m_judge.getScore() + getCrushPoints() + m_stuntScore;
}

std::string Game::hudString() const
{
    char message[64] = "";
    if (m_wrecked)
    {
        std::snprintf(message, sizeof(message), "WRECKED - PRESS R");
    }
    else if (m_judge.isAirborne())
    {
        std::snprintf(message, sizeof(message), "AIR %.1f s", m_judge.getAirTime());
    }
    else if (m_judge.getLastRating() != Judge::Rating::None && m_judge.getMessageClock() < m_messageDuration)
    {
        if (m_judge.getLastFlips() > 0)
        {
            std::snprintf(message, sizeof(message), "%s +%d (%d FLIP%s)",
                Judge::ratingName(m_judge.getLastRating()), m_judge.getLastPoints(),
                m_judge.getLastFlips(), m_judge.getLastFlips() > 1 ? "S" : "");
        }
        else
        {
            std::snprintf(message, sizeof(message), "%s +%d",
                Judge::ratingName(m_judge.getLastRating()), m_judge.getLastPoints());
        }
    }
    else if (m_loopClock >= 0.0 && m_loopClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "LOOP! +%d", m_loopPoints);
    }
    else if (m_lapClock >= 0.0 && m_lapClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "LAP %.1f s", m_lastLapTime);
    }
    else if (m_crushClock >= 0.0 && m_crushClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "CRUSHED! +%d", m_crushPoints + m_crushBonus);
    }

    char hud[160];
    std::snprintf(hud, sizeof(hud), "%d|%d|%d|%s|%d|%.1f|%.1f",
        (int) std::lround(m_truck.getSpeed() * 3.6), getScore(), m_best, message,
        m_lap, m_lapTime, m_bestLapTime);
    return std::string(hud);
}
