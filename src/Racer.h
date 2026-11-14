// Monster - one truck in the race and everything that belongs to it alone
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Color.h"
#include "Driver.h"
#include "Judge.h"
#include "Truck.h"

#include <string>
#include <vector>

// Struct Racer
// The state of one competitor. The rules that advance it live in Game, which owns the circuit
// and the tuning they depend on.
struct Racer
{
    // How far the truck has got with a gate. It has to come up to the gate and then leave it on
    // the far side, so that driving past the back of one does not clear it.
    enum class Crossing
    {
        Away,
        Approached,
        Cleared
    };

    Racer(const std::string& name, const Color& color);                             // driven by the player
    Racer(const std::string& name, const Color& color, const Driver::Style& style); // driven by the computer

    // Mutator
    void clearCheckpoints(size_t count); // give up the gates of the lap in progress

    // Accessor
    int getCheckpointsPassed() const;
    bool lapIsComplete() const; // every checkpoint of the current lap has been passed
    int getScore() const;       // jumps and stunts; points for the wreck are kept by the game

    std::string m_name;
    Color m_color;
    Truck m_truck;
    Judge m_judge;
    Controls m_controls;
    Driver m_driver;
    bool m_driven;            // true->the computer drives, false->the player does
    int m_setupIndex;         // tyre setup, kept across a restart

    // Lap
    size_t m_progress;        // index of the last track point reached
    int m_lap;                // 1->first lap
    double m_lapTime;
    double m_bestLapTime;     // negative->no lap finished yet
    double m_lastLapTime;
    std::vector<Crossing> m_crossing; // one per gate, given up every lap

    // Race
    size_t m_gridSlot;        // 0->front of the starting grid
    bool m_finished;          // true->has completed the race distance
    int m_finishPlace;        // 1->winner, 0->not finished
    double m_finishTime;      // race time at the finish [s]

    // Wreck detection
    double m_tippedClock;
    double m_wreckedClock;    // seconds since the truck was wrecked
    bool m_wrecked;           // true->truck is stuck on its side or roof
    double m_stuckClock;      // seconds since a computer driver last made progress
    bool m_detour;            // true->taking the dirt road round the bridge this time
    int m_routeLap;           // lap for which the route was last chosen

    // Messages and stunts
    double m_lapClock;        // seconds since the last lap was finished, negative->none yet
    double m_missedClock;     // seconds since a lap was rejected, negative->none yet
    bool m_loopDone;          // true->loop already scored this lap
    bool m_ahead;             // true->truck is beyond the start/finish line
    double m_loopClock;       // seconds since the loop was scored, negative->not yet
    int m_stuntScore;         // loop points collected this run
    double m_crushShare;      // part of the wreck this truck flattened, 0..1
    bool m_flattened;         // true->this truck finished the wreck off
};
