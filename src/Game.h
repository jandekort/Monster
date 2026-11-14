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
    void recover(); // set the truck upright on the track where it last was
    void startAt(size_t pointIndex); // begin a fresh lap from a given point of the circuit
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
    bool needsRecovery() const { return m_wrecked || m_truck.getPosition().y < m_floor; }
    size_t getProgress() const { return m_progress; } // index of the last track point reached
    int getLap() const { return m_lap; }              // 1->first lap
    double getLapTime() const { return m_lapTime; }
    double getBestLapTime() const { return m_bestLapTime; } // negative->no lap finished yet
    int getCheckpointsPassed() const;
    int getCheckpointCount() const;
    bool lapIsComplete() const; // every checkpoint of the current lap has been passed
    // "speed|score|best|message|lap|lap time|best lap time|checkpoints passed|checkpoints"
    std::string hudString() const;

private:
    void placeOnTrack(size_t index); // upright and at rest on the centreline
    void updateProgress();
    void updateStunts();
    void markCheckpoints(size_t from, size_t to);
    void clearCheckpoints();

    Truck m_truck;
    Terrain m_terrain;
    Judge m_judge;
    Controls m_controls;

    double m_fixedStep;
    double m_maxElapsed;      // longest frame time that is simulated [s]
    double m_accumulator;
    double m_wreckedDelay;    // time on its side or roof before the truck counts as wrecked [s]
    double m_recoverDelay;    // further time before it is set upright on its own [s]
    double m_floor;           // below this height the truck is lost and is recovered [m]
    double m_uprightNormal;   // lowest surface normal y that can be recovered onto
    double m_messageDuration; // how long verdicts stay on screen [s]
    int m_crushPoints;        // points for a completely flattened wreck
    int m_crushBonus;
    int m_loopPoints;         // points for driving through the loop
    double m_gateRadius;      // a track point counts as reached within this distance [m]
    size_t m_gateWindow;      // track points searched ahead of the last one reached

    double m_tippedClock;
    double m_wreckedClock;    // seconds since the truck was wrecked
    bool m_wrecked;           // true->truck is stuck on its side or roof
    double m_crushClock;      // seconds since the wreck was flattened, negative->not yet
    int m_best;
    size_t m_progress;
    int m_lap;
    double m_lapTime;
    double m_bestLapTime;
    std::vector<bool> m_checkpointsPassed; // one flag per checkpoint, cleared every lap
    double m_lapClock;        // seconds since the last lap was finished, negative->none yet
    double m_missedClock;     // seconds since a lap was rejected, negative->none yet
    double m_lastLapTime;
    bool m_loopDone;          // true->loop already scored this lap
    double m_loopClock;       // seconds since the loop was scored, negative->not yet
    int m_stuntScore;         // loop points collected this run
};
