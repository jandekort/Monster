// Monster - the game: one truck, one course, one score
// Author: Jan de Kort
// Created: 2026-11-14
#include "Game.h"

#include "Check.h"

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
    , m_recoverDelay(1.5)
    , m_floor(-5.0)
    , m_uprightNormal(0.5)
    , m_messageDuration(3.0)
    , m_crushPoints(300)
    , m_crushBonus(200)
    , m_loopPoints(500)
    , m_gateRadius(12.0)
    , m_gateWindow(10)
    , m_tippedClock(0.0)
    , m_wreckedClock(0.0)
    , m_wrecked(false)
    , m_crushClock(-1.0)
    , m_best(0)
    , m_progress(0)
    , m_lap(1)
    , m_lapTime(0.0)
    , m_bestLapTime(-1.0)
    , m_checkpointsPassed()
    , m_lapClock(-1.0)
    , m_missedClock(-1.0)
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
    m_wreckedClock = 0.0;
    m_wrecked = false;
    m_crushClock = -1.0;
    m_lap = 1;
    m_lapTime = 0.0;
    m_lapClock = -1.0;
    m_missedClock = -1.0;
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
    clearCheckpoints();
}

void Game::placeOnTrack(size_t index)
{
    const std::vector<PathPoint>& points = m_terrain.getTrack().getPoints();

    // The loop is skipped: standing the truck on its ceiling would only drop it again
    while (index > 0 && points[index].m_normal.y < m_uprightNormal)
    {
        --index;
    }

    const PathPoint& point = points[index];
    m_progress = index;
    m_truck.placeAt(point.m_position + point.m_normal * m_truck.getRideHeight(),
        point.m_forward, point.m_normal);
    m_judge.settle();
    m_tippedClock = 0.0;
    m_wreckedClock = 0.0;
    m_wrecked = false;
}

// Begin again from somewhere on the circuit. The checkpoints of the lap in progress are given
// up, because the truck did not drive past them, so that lap can no longer be credited.
void Game::startAt(size_t pointIndex)
{
    require(pointIndex < m_terrain.getTrack().getPoints().size(), "Game::startAt: no such track point");
    placeOnTrack(pointIndex);
    clearCheckpoints();
    m_lapTime = 0.0;
}

// Put the truck back on the road where it last was. The checkpoints already passed this lap
// are kept: being set upright is not a shortcut.
void Game::recover()
{
    placeOnTrack(m_progress);
}

void Game::step(double elapsed)
{
    m_accumulator += clamp(elapsed, 0.0, m_maxElapsed);
    while (m_accumulator >= m_fixedStep)
    {
        m_accumulator -= m_fixedStep;

        // A wrecked truck ignores the controls, so that pressing a key cannot flip it back
        // onto its wheels and launch it
        m_truck.step(m_fixedStep, m_wrecked ? Controls() : m_controls, m_terrain);
        m_judge.update(m_truck, m_fixedStep);
        updateProgress();
        updateStunts();

        { // Wrecked when lying on its side or roof for a while
            const bool tipped = m_truck.up().y < 0.3 && m_truck.getSpeed() < 2.0;
            m_tippedClock = tipped ? m_tippedClock + m_fixedStep : 0.0;
            m_wrecked = m_tippedClock > m_wreckedDelay;
            m_wreckedClock = m_wrecked ? m_wreckedClock + m_fixedStep : 0.0;
        }

        if (m_wreckedClock > m_recoverDelay || m_truck.getPosition().y < m_floor)
        {
            recover();
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

int Game::getCheckpointCount() const
{
    return (int) m_terrain.getTrack().getCheckpoints().size();
}

int Game::getCheckpointsPassed() const
{
    return (int) std::count(m_checkpointsPassed.begin(), m_checkpointsPassed.end(), true);
}

bool Game::lapIsComplete() const
{
    return std::all_of(m_checkpointsPassed.begin(), m_checkpointsPassed.end(),
        [](bool passed) { return passed; });
}

void Game::clearCheckpoints()
{
    m_checkpointsPassed.assign(m_terrain.getTrack().getCheckpoints().size(), false);
}

// Only gates the truck actually drove through are marked, so a lap begun partway round the
// circuit cannot be credited
void Game::markCheckpoints(size_t from, size_t to)
{
    const std::vector<size_t>& checkpoints = m_terrain.getTrack().getCheckpoints();
    for (size_t i = 0; i < checkpoints.size(); ++i)
    {
        if (checkpoints[i] > from && checkpoints[i] <= to)
        {
            m_checkpointsPassed[i] = true;
        }
    }
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
    if (m_missedClock >= 0.0)
    {
        m_missedClock += m_fixedStep;
    }

    const size_t previous = m_progress;
    for (size_t ahead = m_gateWindow; ahead >= 1; --ahead)
    {
        const size_t index = std::min(m_progress + ahead, last);
        if (index > m_progress && length(points[index].m_position - m_truck.getPosition()) < m_gateRadius)
        {
            m_progress = index;
            break;
        }
    }

    markCheckpoints(previous, m_progress);

    if (m_progress == last)
    {
        const bool complete = lapIsComplete();
        m_progress = 0;
        clearCheckpoints();
        m_loopDone = false;
        if (complete)
        {
            m_lastLapTime = m_lapTime;
            m_bestLapTime = m_bestLapTime < 0.0 ? m_lapTime : std::min(m_bestLapTime, m_lapTime);
            m_lapClock = 0.0;
            ++m_lap;
        }
        else
        {
            m_missedClock = 0.0;
        }
        m_lapTime = 0.0;
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
        std::snprintf(message, sizeof(message), "WRECKED - SPACE TO RECOVER");
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
    else if (m_missedClock >= 0.0 && m_missedClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "LAP NOT COUNTED - CHECKPOINT MISSED");
    }
    else if (m_crushClock >= 0.0 && m_crushClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "CRUSHED! +%d", m_crushPoints + m_crushBonus);
    }

    char hud[224];
    std::snprintf(hud, sizeof(hud), "%d|%d|%d|%s|%d|%.1f|%.1f|%d|%d",
        (int) std::lround(m_truck.getSpeed() * 3.6), getScore(), m_best, message,
        m_lap, m_lapTime, m_bestLapTime, getCheckpointsPassed(), getCheckpointCount());
    return std::string(hud);
}
