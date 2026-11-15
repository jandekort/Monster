// Monster - one truck in the race and everything that belongs to it alone
// Author: Jan de Kort
// Created: 2026-11-14
#include "Racer.h"

#include <algorithm>

Racer::Racer(const std::string& name, const Color& color)
    :
    Racer(name, color, Driver::Style())
{
    m_driven = false;
}

Racer::Racer(const std::string& name, const Color& color, const Driver::Style& style)
    :
    m_name(name)
    , m_color(color)
    , m_truck()
    , m_judge()
    , m_controls()
    , m_driver(style)
    , m_driven(true)
    , m_setupIndex(3)
    , m_progress(0)
    , m_lap(1)
    , m_lapTime(0.0)
    , m_bestLapTime(-1.0)
    , m_lastLapTime(0.0)
    , m_crossing()
    , m_gridSlot(0)
    , m_finished(false)
    , m_finishPlace(0)
    , m_finishTime(0.0)
    , m_tippedClock(0.0)
    , m_wreckedClock(0.0)
    , m_wrecked(false)
    , m_stuckClock(0.0)
    , m_detour(-1)
    , m_routeChoice(-1)
    , m_shaken(false)
    , m_gateClock(0.0)
    , m_lapClock(-1.0)
    , m_missedClock(-1.0)
    , m_loopDone(false)
    , m_ahead(false)
    , m_loopClock(-1.0)
    , m_stuntScore(0)
    , m_crushShare(0.0)
    , m_flattened(false)
{
}

void Racer::clearCheckpoints(size_t count)
{
    m_crossing.assign(count, Crossing::Away);
}

int Racer::getCheckpointsPassed() const
{
    return (int) std::count(m_crossing.begin(), m_crossing.end(), Crossing::Cleared);
}

bool Racer::lapIsComplete() const
{
    return std::all_of(m_crossing.begin(), m_crossing.end(),
        [](Crossing crossing) { return crossing == Crossing::Cleared; });
}

int Racer::getScore() const
{
    return m_judge.getScore() + m_stuntScore;
}
