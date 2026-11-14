// Monster - the game: one truck, one course, one score
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Judge.h"
#include "Terrain.h"
#include "Truck.h"

#include <string>

// Class Game
class Game
{
public:
    Game();

    // Mutator
    void reset();
    void step(double elapsed); // advances the simulation in fixed steps
    void setControls(const Controls& controls) { m_controls = controls; }

    // Accessor
    const Truck& getTruck() const { return m_truck; }
    const Terrain& getTerrain() const { return m_terrain; }
    const Judge& getJudge() const { return m_judge; }
    int getScore() const;
    int getBestScore() const { return m_best; }
    int getCrushPoints() const;
    bool isWrecked() const { return m_wrecked; }
    size_t getProgress() const { return m_progress; } // index of the last track point reached
    int getLap() const { return m_lap; }              // 1->first lap
    double getLapTime() const { return m_lapTime; }
    double getBestLapTime() const { return m_bestLapTime; } // negative->no lap finished yet
    std::string hudString() const; // "speed|score|best|message|lap|lap time|best lap time"

private:
    void updateProgress();
    void updateStunts();

    Truck m_truck;
    Terrain m_terrain;
    Judge m_judge;
    Controls m_controls;

    double m_fixedStep;
    double m_maxElapsed;      // longest frame time that is simulated [s]
    double m_accumulator;
    double m_wreckedDelay;    // time on its side or roof before the run ends [s]
    double m_messageDuration; // how long verdicts stay on screen [s]
    int m_crushPoints;        // points for a completely flattened wreck
    int m_crushBonus;
    int m_loopPoints;         // points for driving through the loop
    double m_gateRadius;      // a track point counts as reached within this distance [m]
    size_t m_gateWindow;      // track points searched ahead of the last one reached

    double m_tippedClock;
    bool m_wrecked;           // true->truck is stuck on its side or roof
    double m_crushClock;      // seconds since the wreck was flattened, negative->not yet
    int m_best;
    size_t m_progress;
    int m_lap;
    double m_lapTime;
    double m_bestLapTime;
    double m_lapClock;        // seconds since the last lap was finished, negative->none yet
    double m_lastLapTime;
    bool m_loopDone;          // true->loop already scored this lap
    double m_loopClock;       // seconds since the loop was scored, negative->not yet
    int m_stuntScore;         // loop points collected this run
};
