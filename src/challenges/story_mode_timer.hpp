//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2006-2019 SuperTuxKart-Team
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#ifndef HEADER_STORY_MODE_TIMER_HPP
#define HEADER_STORY_MODE_TIMER_HPP

#include "utils/types.hpp"
#include <iomanip>

class StoryModeTimer
{
private:
    bool m_valid_speedrun_started, m_valid_speedrun_ended;
    bool m_story_mode_started, m_story_mode_ended;
    bool m_speedrun_pause_active, m_story_mode_pause_active;
    bool m_loading_pause;

    //This stores the number of milliseconds to display with the counter
    int m_speedrun_milliseconds;
    int m_stored_speedrun_milliseconds;

    uint64_t m_speedrun_start;
    uint64_t m_speedrun_end;
    uint64_t m_speedrun_pause_start;

    uint64_t m_speedrun_total_pause_time;

    void testPlayerRun();

    void pauseSpeedrunTimer();
    void unpauseSpeedrunTimer();
    void updateSpeedrunTimer();
public:

    StoryModeTimer();

    // ------------------------------------------------------------------------
    /** Speedrun timer functions. */
    void startTimer();
    void stopTimer();
    void pauseTimer(bool loading);
    void unpauseTimer(bool loading);
    void updateTimer();
    void reset();

    std::string getTimerString();
    bool isStoryModePaused() const { return m_story_mode_pause_active; }
    bool isSpeedrunning() const { return m_valid_speedrun_started; }
    bool speedrunIsFinished() const { return m_valid_speedrun_ended; }
    int getSpeedrunTime() const { return m_speedrun_milliseconds; }
};   // StoryModeTimer

extern StoryModeTimer* story_mode_timer;

#endif

/* EOF */
