//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2019-present Fouks
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

#ifndef HEADER_TAS_HPP
#define HEADER_TAS_HPP

#include <array>
#include <cassert>
#include <iomanip>
#include <memory>
#include <vector>

// Stores an Input, simulates Key Presses.
struct TasInput
{
	bool a, d, l, r, s, n, f, b, t; // Accelerate, slow Down, steer Left/Right, Skid, Nitro, Fire, look Back, Thunderbird.

	TasInput() : a(false), d(false), l(false), r(false), s(false), n(false), f(false), b(false), t(false) {}

	std::string toString() const;
	std::string toStringConstSize() const;
	bool parse(std::string);
};

// Class for TAS tools
class Tas {
	static Tas *m_tas;

	bool m_is_enabled;
	enum Status {PAUSED, TICK_ADVANCE, NORMAL} m_status;
	bool m_is_paused, m_restart_requested, m_is_recording_frames;

	uint64_t m_current_tick, m_checkpoint, m_frame_number;

	std::string m_inputs_filename, m_bfname;
	std::vector<TasInput> m_file_inputs, m_inputs;
	std::array<int, 4> m_bfv; // Brute Force Variables.
	
	double m_prevx, m_prevy, m_prevz, m_prevv;
	double m_x, m_y, m_z, m_v;

public:
	Tas() :
		m_is_enabled(false),
		m_status(Status::NORMAL),
		m_is_paused(false), m_restart_requested(false), m_is_recording_frames(false),
		m_current_tick(0ULL), m_checkpoint(0ULL), m_frame_number(0ULL),
		m_inputs_filename(""), m_bfname(""),
		m_prevx(0.), m_prevy(0.), m_prevz(0.), m_prevv(0.),
		m_x(0.), m_y(0.), m_z(0.), m_v(0.) {}
	
	// Handle Static Pointer to the one instance of the Tas object.
	static void create() {assert(!m_tas); m_tas = new Tas();}
	static Tas *get() {return m_tas;}
	static void destroy() {delete m_tas; m_tas = NULL;}

	// Only Enabled with --tas Option.
	void enable();
	bool isEnabled() {return m_is_enabled;}
	
	// Press P to Pause/Unpause, and O to Advance one Tick.
	bool isPaused() {return m_status == Status::PAUSED;}
	bool isTickAdvancing() {return m_status == Status::TICK_ADVANCE;}
	void pause();
	void tickAdvance();
	void resume();

	// No Inputs during Loading Screens.
	void pauseReplay() {
		if (!m_is_enabled) return;
		m_is_paused = true;
	}
	void unpauseReplay() {
		if (!m_is_enabled) return;
		m_is_paused = false;
	}
	
	// Frames Recording, in order to assemble them later to make a Video. Press L to start/stop.
	bool isRecordingFrames() const {return m_is_recording_frames;}
	void saveFrame();
	void startRecordingFrames();
	void stopRecordingFrames();

	// Load Inputs from File and Update when restarting (read again the File or try next Input Combination when Brute Forcing).
	bool loadInputs(const std::string&);
	bool updateInputs();
	TasInput getCurrentInput() const {
		if (m_current_tick >= m_inputs.size())
			return TasInput();
		return m_inputs[m_current_tick];
	}
	TasInput getPreviousInput() const {
		if (m_current_tick < 1 || m_current_tick > m_inputs.size())
			return TasInput();
		return m_inputs[m_current_tick];
	}
	
	// Apply Current Input or change its Index.
	void update(const double, const double, const double, const double); // Also Update Kart Position/Speed
	uint64_t currentTick() const {return m_current_tick;}
	void setCurrentTick(const uint64_t current_tick) {m_current_tick = current_tick;}
	double currentTimeS() const {return m_current_tick/120.;}
	uint64_t currentTimeMs() const {return (1000ULL*m_current_tick)/120ULL;}

	// Checkpoints.
	uint64_t checkpoint() const {return m_checkpoint;}
	void setCheckpoint();
	void requestReturnToCheckpoint();
	bool restartRequested() {return m_restart_requested;}
	void restartRequestProcessed();
	
	// Brute Forcing.
	void doBruteForceFor(const std::string&);
	bool isBruteForcing() const {return m_bfname != "";}
	bool isCandidate() const;
};

#endif
