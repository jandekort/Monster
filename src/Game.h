// Monster - the game: the circuit, the racers on it and the rules they race by
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Racer.h"
#include "Terrain.h"

#include <string>
#include <vector>

// Class Game
// A race over a fixed number of laps. The racer at index 0 is the player and the rest are driven
// by the computer; with no opponents the player has the circuit alone. The accessors without a
// racer argument describe the player.
class Game
{
public:
    explicit Game(size_t opponents);

    static size_t maxOpponents();

    // Mutator
    void reset();
    void recover(); // set the player's truck upright at the last checkpoint it cleared
    void startAt(size_t pointIndex); // begin a fresh lap from a given point of the circuit
    void step(double elapsed); // advances the simulation in fixed steps
    void setControls(const Controls& controls) { player().m_controls = controls; }
    void selectSetup(int index); // tyre setup, 1 is the softest; out of range values are clamped

    // Accessor
    const std::vector<Racer>& getRacers() const { return m_racers; }
    const Racer& getPlayer() const { return m_racers.front(); }
    const Terrain& getTerrain() const { return m_terrain; }

    const Truck& getTruck() const { return getPlayer().m_truck; }
    const Judge& getJudge() const { return getPlayer().m_judge; }
    int getScore() const;
    int getScore(const Racer& racer) const; // jumps, stunts and this racer's share of the wreck
    int getBestScore() const { return m_best; }
    int getCrushPoints() const;
    int getCrushPoints(const Racer& racer) const;
    bool isWrecked() const { return getPlayer().m_wrecked; }
    bool needsRecovery() const { return isWrecked() || getTruck().getPosition().y < m_floor; }
    size_t getProgress() const { return getPlayer().m_progress; } // index of the last track point reached
    int getLap() const { return getPlayer().m_lap; }              // 1->first lap
    double getLapTime() const { return getPlayer().m_lapTime; }
    double getBestLapTime() const { return getPlayer().m_bestLapTime; } // negative->no lap finished yet
    int getSetupIndex() const { return getPlayer().m_setupIndex; }
    const char* getSetupName() const { return Truck::setupName(getSetupIndex()); }
    int getCheckpointsPassed() const { return getPlayer().getCheckpointsPassed(); }
    int getCheckpointCount() const;
    bool lapIsComplete() const { return getPlayer().lapIsComplete(); }

    // Race
    int getRaceLaps() const { return m_raceLaps; }
    double getRaceTime() const { return m_raceClock; }
    bool isRaceOver() const { return getPlayer().m_finished; } // the race ends when the player finishes
    int getPosition() const { return getPosition(getPlayer()); }
    int getPosition(const Racer& racer) const; // 1->leading, as settled at its last gate or the line
    // "speed|score|best|message|lap|lap time|best lap time|checkpoints passed|checkpoints"
    // "|tyre setup|setup name|grip|top speed in km/h|position|racers|race laps|finished""
    std::string hudString() const;

private:
    Racer& player() { return m_racers.front(); }

    // The rules, one racer at a time
    void resetRacer(Racer& racer);
    void placeOnGrid(Racer& racer);
    void collideRacers();
    bool isAhead(const Racer& a, const Racer& b) const;
    int gatesBehind(const Racer& racer) const; // gates cleared since the start, laps included
    void settlePlace(Racer& racer, int gate);  // as it clears a gate (1->the first) or the line (0)
    void updateStuck(Racer& racer, size_t progressBefore);
    void chooseRoute(Racer& racer);
    const Detour* detourOf(const Racer& racer) const; // the dirt road a racer is on, or none
    void stepRacer(Racer& racer);
    void placeOnTrack(Racer& racer, size_t index); // upright and at rest on the centreline
    void recover(Racer& racer);
    void updateCheckpoints(Racer& racer);
    void updateProgress(Racer& racer);
    void updateStunts(Racer& racer);
    void updateWrecked(Racer& racer);

    std::vector<Racer> m_racers;
    Terrain m_terrain;

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
    double m_gateDepth;       // how far either side of a gate its two lines lie [m]
    double m_gateReach;       // the truck must be this near a gate to be crossing it [m]
    int m_raceLaps;
    double m_gridFront;       // front row behind the start line [m]
    double m_gridRow;         // one grid slot behind the next [m]
    double m_gridLane;        // the two columns stand this far either side of the centreline [m]
    double m_stuckDelay;      // a computer driver making no progress this long is recovered [s]
    size_t m_routeWindow;     // track points before a junction over which the route is chosen
    double m_missedGrace;     // a lap that does not count is not announced this early in the race [s]
    double m_placeGrace;      // nor is the place at a gate or the line [s]
    unsigned m_dice;          // state of the generator the route choices are drawn from

    double m_raceClock;       // seconds since the start
    int m_finishers;          // racers that have completed the race distance

    double m_crushClock;      // seconds since the wreck was flattened, negative->not yet
    int m_best;               // the player's best score over all runs
};
