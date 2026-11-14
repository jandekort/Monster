// Monster - the game: the circuit, the racers on it and the rules they race by
// Author: Jan de Kort
// Created: 2026-11-14
#include "Game.h"

#include "Check.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
    // Struct Rival: one of the computer driven opponents
    struct Rival
    {
        const char* m_name;
        Color m_color;
        Driver::Style m_style;
    };

    //                                                 look  gain  corner  brake  plan  throttle
    const Rival g_rivals[] = {
        { "BLUE",   Color(0.15f, 0.35f, 0.85f), { 14.0,  2.5,  12.5,   5.0,   80.0,  1.00 } },
        { "YELLOW", Color(0.95f, 0.78f, 0.10f), { 14.0,  2.5,  11.0,   4.0,   80.0,  0.90 } },
        { "GREEN",  Color(0.15f, 0.62f, 0.25f), { 12.0,  2.5,   9.5,   3.5,   80.0,  0.80 } }
    };
}

size_t Game::maxOpponents()
{
    return sizeof(g_rivals) / sizeof(g_rivals[0]);
}

Game::Game(size_t opponents)
    :
    m_racers(1, Racer("PLAYER", Color(0.85f, 0.12f, 0.10f)))
    , m_terrain()
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
    , m_gateDepth(5.0)
    , m_gateReach(40.0)
    , m_raceLaps(3)
    , m_gridFront(7.0)
    , m_gridRow(5.5)
    , m_gridLane(2.5)
    , m_stuckDelay(6.0)
    , m_raceClock(0.0)
    , m_finishers(0)
    , m_crushClock(-1.0)
    , m_best(0)
{
    require(opponents <= maxOpponents(), "Game: too many opponents");

    // The opponents take the front of the grid and the player starts behind them
    m_racers.reserve(opponents + 1);
    for (size_t i = 0; i < opponents; ++i)
    {
        m_racers.push_back(Racer(g_rivals[i].m_name, g_rivals[i].m_color, g_rivals[i].m_style));
        m_racers.back().m_gridSlot = i;
    }
    player().m_gridSlot = opponents;

    reset();
}

void Game::selectSetup(int index)
{
    Racer& racer = player();
    racer.m_setupIndex = (int) clamp(index, 1.0, Truck::setupCount());
    racer.m_truck.setSetup(Truck::setupAt(racer.m_setupIndex));
}

void Game::reset()
{
    m_best = std::max(m_best, getScore());
    m_terrain.reset();
    m_accumulator = 0.0;
    m_crushClock = -1.0;
    m_raceClock = 0.0;
    m_finishers = 0;
    for (Racer& racer : m_racers)
    {
        resetRacer(racer);
    }
}

void Game::resetRacer(Racer& racer)
{
    racer.m_truck.setSetup(Truck::setupAt(racer.m_setupIndex));
    racer.m_judge.reset();
    placeOnGrid(racer);
    racer.m_tippedClock = 0.0;
    racer.m_wreckedClock = 0.0;
    racer.m_wrecked = false;
    racer.m_lap = 1;
    racer.m_lapTime = 0.0;
    racer.m_lapClock = -1.0;
    racer.m_missedClock = -1.0;
    racer.m_loopDone = false;
    racer.m_loopClock = -1.0;
    racer.m_stuntScore = 0;
    racer.m_crushShare = 0.0;
    racer.m_flattened = false;
    racer.m_stuckClock = 0.0;
    racer.m_finished = false;
    racer.m_finishPlace = 0;
    racer.m_finishTime = 0.0;
    racer.m_progress = 0; // the grid is behind the line: the first track point is still ahead
    racer.clearCheckpoints(m_terrain.getTrack().getCheckpoints().size());
}

// A staggered grid on the straight behind the start line: two columns, each slot half a truck
// behind the one before. A truck with the circuit to itself starts in the middle of the road.
void Game::placeOnGrid(Racer& racer)
{
    const Track& track = m_terrain.getTrack();
    const PathPoint& line = track.getStartFinish();
    const double back = m_gridFront + m_gridRow * racer.m_gridSlot;
    const double side = m_racers.size() == 1 ? 0.0 : (racer.m_gridSlot % 2 == 0 ? -m_gridLane : m_gridLane);

    { // The grid is laid out straight back from the line, so the last straight must hold it
        const std::vector<PathPoint>& points = track.getPoints();
        const bool straight = std::none_of(points.begin(), points.end(),
            [&](const PathPoint& point)
            {
                return point.m_corner && track.getLength() - point.m_distance < back + 3.0;
            });
        require(straight, "Game::placeOnGrid: the grid does not fit on the last straight");
    }

    racer.m_truck.placeAt(line.m_position - line.m_forward * back + line.m_cross * side
        + line.m_normal * racer.m_truck.getRideHeight(), line.m_forward, line.m_normal);
}

void Game::placeOnTrack(Racer& racer, size_t index)
{
    const std::vector<PathPoint>& points = m_terrain.getTrack().getPoints();

    // The loop is skipped: standing the truck on its ceiling would only drop it again
    while (index > 0 && points[index].m_normal.y < m_uprightNormal)
    {
        --index;
    }

    const PathPoint& point = points[index];
    racer.m_progress = index;
    racer.m_truck.placeAt(point.m_position + point.m_normal * racer.m_truck.getRideHeight(),
        point.m_forward, point.m_normal);
    racer.m_judge.settle();
    racer.m_tippedClock = 0.0;
    racer.m_wreckedClock = 0.0;
    racer.m_wrecked = false;
    racer.m_stuckClock = 0.0;
}

// Begin again from somewhere on the circuit. The checkpoints of the lap in progress are given
// up, because the truck did not drive past them, so that lap can no longer be credited.
void Game::startAt(size_t pointIndex)
{
    require(pointIndex < m_terrain.getTrack().getPoints().size(), "Game::startAt: no such track point");
    Racer& racer = player();
    placeOnTrack(racer, pointIndex);
    racer.clearCheckpoints(m_terrain.getTrack().getCheckpoints().size());
    racer.m_lapTime = 0.0;
}

void Game::recover()
{
    recover(player());
}

// Put the truck back on the road at the last gate it cleared, or at the start and finish line
// if it has not cleared one yet. Gates already cleared are kept: they were driven through.
void Game::recover(Racer& racer)
{
    const std::vector<size_t>& checkpoints = m_terrain.getTrack().getCheckpoints();
    size_t target = 0;
    for (size_t i = 0; i < checkpoints.size(); ++i)
    {
        if (racer.m_crossing[i] == Racer::Crossing::Cleared)
        {
            target = checkpoints[i];
        }
    }
    placeOnTrack(racer, target);
}

void Game::step(double elapsed)
{
    m_accumulator += clamp(elapsed, 0.0, m_maxElapsed);
    while (m_accumulator >= m_fixedStep)
    {
        m_accumulator -= m_fixedStep;
        m_raceClock += m_fixedStep;

        collideRacers();
        for (Racer& racer : m_racers)
        {
            stepRacer(racer);
        }

        { // Crush message
            if (m_crushClock >= 0.0)
            {
                m_crushClock += m_fixedStep;
            }
            else if (m_terrain.getCrush() >= 1.0)
            {
                m_crushClock = 0.0;
            }
        }
    }
    m_best = std::max(m_best, getScore());
}

// Trucks are not part of the ray cast world, so contact between them is settled here, pair by
// pair, before any of them moves
void Game::collideRacers()
{
    for (size_t i = 0; i < m_racers.size(); ++i)
    {
        for (size_t j = i + 1; j < m_racers.size(); ++j)
        {
            Truck::collide(m_racers[i].m_truck, m_racers[j].m_truck);
        }
    }
}

void Game::stepRacer(Racer& racer)
{
    if (racer.m_driven)
    {
        racer.m_controls = racer.m_driver.drive(racer.m_truck, racer.m_progress, m_terrain.getTrack());
    }

    Controls controls = racer.m_controls;
    if (racer.m_wrecked)
    {
        // A wrecked truck ignores the controls, so that pressing a key cannot flip it back onto
        // its wheels and launch it
        controls = Controls();
    }
    else if (racer.m_finished && !racer.m_driven)
    {
        // The race is over for the player: the truck is brought to a stop
        controls = Controls();
        controls.m_brake = dot(racer.m_truck.getVelocity(), racer.m_truck.forward()) > 1.0 ? 1.0 : 0.0;
    }

    const size_t progressBefore = racer.m_progress;
    const double crushBefore = m_terrain.getCrush();
    racer.m_truck.step(m_fixedStep, controls, m_terrain);

    { // Whoever flattens the wreck scores for it
        const double crushAfter = m_terrain.getCrush();
        racer.m_crushShare += crushAfter - crushBefore;
        racer.m_flattened = racer.m_flattened || (crushBefore < 1.0 && crushAfter >= 1.0);
    }

    racer.m_judge.update(racer.m_truck, m_fixedStep);
    updateCheckpoints(racer);
    updateProgress(racer);
    updateStunts(racer);
    updateWrecked(racer);
    updateStuck(racer, progressBefore);

    if (racer.m_wreckedClock > m_recoverDelay || racer.m_truck.getPosition().y < m_floor)
    {
        recover(racer);
    }
}

// A computer driver that stops getting anywhere, against a wall or at the foot of the loop, is
// put back on the road the way the player would do it with a key
void Game::updateStuck(Racer& racer, size_t progressBefore)
{
    if (!racer.m_driven)
    {
        return;
    }
    racer.m_stuckClock = racer.m_progress != progressBefore ? 0.0 : racer.m_stuckClock + m_fixedStep;
    if (racer.m_stuckClock > m_stuckDelay)
    {
        recover(racer);
    }
}

// Wrecked when lying on its side or roof for a while
void Game::updateWrecked(Racer& racer)
{
    const bool tipped = racer.m_truck.up().y < 0.3 && racer.m_truck.getSpeed() < 2.0;
    racer.m_tippedClock = tipped ? racer.m_tippedClock + m_fixedStep : 0.0;
    racer.m_wrecked = racer.m_tippedClock > m_wreckedDelay;
    racer.m_wreckedClock = racer.m_wrecked ? racer.m_wreckedClock + m_fixedStep : 0.0;
}

int Game::getCheckpointCount() const
{
    return (int) m_terrain.getTrack().getCheckpoints().size();
}

// A gate is cleared by crossing the line a little before it and then the line a little after it,
// both of them within the width of the gate. Driving round the outside of a gate, or turning
// back short of one, therefore leaves it uncleared.
void Game::updateCheckpoints(Racer& racer)
{
    const Track& track = m_terrain.getTrack();
    const std::vector<PathPoint>& points = track.getPoints();
    const std::vector<size_t>& checkpoints = track.getCheckpoints();

    for (size_t i = 0; i < checkpoints.size(); ++i)
    {
        Racer::Crossing& crossing = racer.m_crossing[i];
        if (crossing == Racer::Crossing::Cleared)
        {
            continue;
        }

        const PathPoint& gate = points[checkpoints[i]];
        const Vec3 offset = racer.m_truck.getPosition() - gate.m_position;
        if (length(offset) > m_gateReach || std::fabs(dot(offset, gate.m_cross)) > track.getGateHalfWidth())
        {
            continue;
        }

        const double along = dot(offset, gate.m_forward);
        if (crossing == Racer::Crossing::Away && along < -m_gateDepth)
        {
            crossing = Racer::Crossing::Approached;
        }
        else if (crossing == Racer::Crossing::Approached && along > m_gateDepth)
        {
            crossing = Racer::Crossing::Cleared;
        }
    }
}

// Track points must be reached in order, so the loop and the bridge cannot be skipped
void Game::updateProgress(Racer& racer)
{
    const std::vector<PathPoint>& points = m_terrain.getTrack().getPoints();
    const size_t last = points.size() - 1; // coincides with the first point
    racer.m_lapTime += m_fixedStep;
    if (racer.m_lapClock >= 0.0)
    {
        racer.m_lapClock += m_fixedStep;
    }
    if (racer.m_missedClock >= 0.0)
    {
        racer.m_missedClock += m_fixedStep;
    }

    for (size_t ahead = m_gateWindow; ahead >= 1; --ahead)
    {
        const size_t index = std::min(racer.m_progress + ahead, last);
        if (index > racer.m_progress
            && length(points[index].m_position - racer.m_truck.getPosition()) < m_gateRadius)
        {
            racer.m_progress = index;
            break;
        }
    }

    // The lap is credited on crossing the line, not on touching the last track point: taken wide
    // or at speed, the truck can pass the line without ever coming near that one point
    const PathPoint& line = m_terrain.getTrack().getStartFinish();
    const bool roundAgain = racer.m_progress + m_gateWindow >= last
        && dot(racer.m_truck.getPosition() - line.m_position, line.m_forward) > 0.0;

    if (racer.m_progress == last || roundAgain)
    {
        const bool complete = racer.lapIsComplete();
        racer.m_progress = 0;
        racer.clearCheckpoints(m_terrain.getTrack().getCheckpoints().size());
        racer.m_loopDone = false;
        if (complete)
        {
            racer.m_lastLapTime = racer.m_lapTime;
            racer.m_bestLapTime = racer.m_bestLapTime < 0.0
                ? racer.m_lapTime : std::min(racer.m_bestLapTime, racer.m_lapTime);
            racer.m_lapClock = 0.0;
            ++racer.m_lap;
            if (racer.m_lap > m_raceLaps && !racer.m_finished)
            {
                racer.m_finished = true;
                racer.m_finishPlace = ++m_finishers;
                racer.m_finishTime = m_raceClock;
            }
        }
        else
        {
            racer.m_missedClock = 0.0;
        }
        racer.m_lapTime = 0.0;
    }
}

void Game::updateStunts(Racer& racer)
{
    if (racer.m_loopClock >= 0.0)
    {
        racer.m_loopClock += m_fixedStep;
    }

    // Upside down with the wheels on the road: that is the top of the loop
    const bool inverted = racer.m_truck.up().y < -0.7 && racer.m_truck.getWheelsOnGround() >= 2;
    if (inverted && !racer.m_loopDone
        && m_terrain.getTrack().getPoints()[racer.m_progress].m_kind == PathPoint::Kind::Loop)
    {
        racer.m_loopDone = true;
        racer.m_loopClock = 0.0;
        racer.m_stuntScore += m_loopPoints;
    }
}

int Game::getCrushPoints() const
{
    return getCrushPoints(getPlayer());
}

// Points for the part of the wreck this racer flattened, and the bonus for finishing it off
int Game::getCrushPoints(const Racer& racer) const
{
    return (int) std::lround(m_crushPoints * clamp(racer.m_crushShare, 0.0, 1.0))
        + (racer.m_flattened ? m_crushBonus : 0);
}

int Game::getScore() const
{
    return getScore(getPlayer());
}

int Game::getScore(const Racer& racer) const
{
    return racer.getScore() + getCrushPoints(racer);
}

// Finishers lead in the order they finished. The rest are ranked by laps and then by how far
// round the lap they are; on the grid, where nobody has got anywhere, by grid slot.
bool Game::isAhead(const Racer& a, const Racer& b) const
{
    if (a.m_finished || b.m_finished)
    {
        return a.m_finished && (!b.m_finished || a.m_finishPlace < b.m_finishPlace);
    }
    const size_t lapPoints = m_terrain.getTrack().getPoints().size();
    const size_t ground = a.m_lap * lapPoints + a.m_progress;
    const size_t rival = b.m_lap * lapPoints + b.m_progress;
    return ground != rival ? ground > rival : a.m_gridSlot < b.m_gridSlot;
}

int Game::getPosition(const Racer& racer) const
{
    return 1 + (int) std::count_if(m_racers.begin(), m_racers.end(),
        [&](const Racer& other) { return &other != &racer && isAhead(other, racer); });
}

std::string Game::hudString() const
{
    const Racer& racer = getPlayer();
    const Judge& judge = racer.m_judge;

    char message[64] = "";
    if (racer.m_finished)
    {
        std::snprintf(message, sizeof(message), "FINISHED P%d/%d - PRESS R TO RACE AGAIN",
            racer.m_finishPlace, (int) m_racers.size());
    }
    else if (racer.m_wrecked)
    {
        std::snprintf(message, sizeof(message), "WRECKED - SPACE TO RECOVER");
    }
    else if (judge.isAirborne())
    {
        std::snprintf(message, sizeof(message), "AIR %.1f s", judge.getAirTime());
    }
    else if (judge.getLastRating() != Judge::Rating::None && judge.getMessageClock() < m_messageDuration)
    {
        if (judge.getLastFlips() > 0)
        {
            std::snprintf(message, sizeof(message), "%s +%d (%d FLIP%s)",
                Judge::ratingName(judge.getLastRating()), judge.getLastPoints(),
                judge.getLastFlips(), judge.getLastFlips() > 1 ? "S" : "");
        }
        else
        {
            std::snprintf(message, sizeof(message), "%s +%d",
                Judge::ratingName(judge.getLastRating()), judge.getLastPoints());
        }
    }
    else if (racer.m_loopClock >= 0.0 && racer.m_loopClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "LOOP! +%d", m_loopPoints);
    }
    else if (racer.m_lapClock >= 0.0 && racer.m_lapClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "LAP %.1f s", racer.m_lastLapTime);
    }
    else if (racer.m_missedClock >= 0.0 && racer.m_missedClock < m_messageDuration)
    {
        std::snprintf(message, sizeof(message), "LAP NOT COUNTED - CHECKPOINT MISSED");
    }
    else if (m_crushClock >= 0.0 && m_crushClock < m_messageDuration && getCrushPoints(racer) > 0)
    {
        std::snprintf(message, sizeof(message), "CRUSHED! +%d", getCrushPoints(racer));
    }

    const Truck::Setup& setup = racer.m_truck.getSetup();
    char hud[320];
    std::snprintf(hud, sizeof(hud), "%d|%d|%d|%s|%d|%.1f|%.1f|%d|%d|%d|%s|%.2f|%d|%d|%d|%d|%d",
        (int) std::lround(racer.m_truck.getSpeed() * 3.6), getScore(), m_best, message,
        std::min(racer.m_lap, m_raceLaps), racer.m_lapTime, racer.m_bestLapTime,
        racer.getCheckpointsPassed(), getCheckpointCount(),
        racer.m_setupIndex, getSetupName(), setup.m_grip, (int) std::lround(setup.m_topSpeed * 3.6),
        getPosition(racer), (int) m_racers.size(), m_raceLaps, racer.m_finished ? 1 : 0);
    return std::string(hud);
}
