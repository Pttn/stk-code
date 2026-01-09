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

#include <iostream>
#include <fstream>
#include <sstream>

#include "graphics/irr_driver.hpp"
#include "guiengine/message_queue.hpp"
#include "karts/abstract_kart.hpp"
#include "karts/controller/player_controller.hpp"
#include "karts/kart.hpp"
#include "modes/linear_world.hpp"
#include "tas.hpp"
#include "tracks/track.hpp"
#include "utils/log.hpp"
#include "utils/time.hpp"
#include "input/input_manager.hpp"

std::string TasInput::toString() const {
	std::ostringstream oss;
	oss << (a ? "a" : "") << (d ? "d" : "") << (l ? "l" : "") << (r ? "r" : "") << (s ? "s" : "") << (n ? "n" : "") << (f ? "f" : "") << (b ? "b" : "") << (t ? "t" : "");
	return oss.str();
}
std::string TasInput::toStringConstSize() const {
	std::ostringstream oss;
	oss << (a ? "a" : " ") << (d ? "d" : " ") << (l ? "l" : " ") << (r ? "r" : " ") << (s ? "s" : " ") << (n ? "n" : " ") << (f ? "f" : " ") << (b ? "b" : " ") << (t ? "t" : " ");
	return oss.str();
}

bool TasInput::parse(std::string line) {
	a = line.contains("a");
	d = line.contains("d");
	l = line.contains("l");
	r = line.contains("r");
	s = line.contains("s");
	n = line.contains("n");
	f = line.contains("f");
	b = line.contains("b");
	t = line.contains("t");
	return true;
}

Tas *Tas::m_tas = nullptr;

void Tas::enable() {
	m_is_enabled = true;
	StkTime::m_mono_start = std::chrono::steady_clock::time_point{std::chrono::seconds{1767225600}};
	Log::info("TAS", "Enabled.");
}

void Tas::pause() {
	Log::info("TAS", "Pausing.");
	m_status = PAUSED;
}
void Tas::tickAdvance() {
	Log::info("TAS", "Tick Advance.");
	m_status = TICK_ADVANCE;
}
void Tas::resume() {
	Log::info("TAS", "Resuming.");
	m_status = NORMAL;
}

void Tas::saveFrame() {
	if (!m_is_enabled) return;
	irr_driver->doFrameShot(m_frame_number);
	m_frame_number++;
}
void Tas::startRecordingFrames() {
	if (!m_is_enabled) return;
	Log::info("TAS", "Start recording frames.");
	m_frame_number = 0;
	m_is_recording_frames = true;
}
void Tas::stopRecordingFrames() {
	if (!m_is_enabled) return;
	char frameNumberStr[7];
	sprintf(frameNumberStr, "%06" PRIu64, m_frame_number);
	Log::info("TAS", (std::string("Stop recording frames at ") + std::string(frameNumberStr)).c_str());
	m_is_recording_frames = false;
	m_frame_number = 0;
}

bool Tas::loadInputs(const std::string &filename) {
	if (!m_is_enabled) return false;
	if (filename.empty()) return true;

	m_inputs_filename = filename;
	m_file_inputs.clear();
	std::ifstream file(m_inputs_filename + ".stktas");
	if (!file) {
		Log::info("TAS", (std::string("Inputs File ") + m_inputs_filename + std::string(".stktas not found, searching in Replay Folder instead...")).c_str());
		file = std::ifstream(file_manager->getReplayDir() + m_inputs_filename + std::string(".stktas"));
		if (!file) {
			Log::warn("TAS", (std::string("Inputs File ") + std::string(file_manager->getReplayDir() + m_inputs_filename) + std::string(".stktas not found either, no TAS replay.")).c_str());
			m_inputs_filename = "";
			return false;
		}
	}

	Log::info("TAS", (std::string("Loading Inputs File ") + std::string(file_manager->getReplayDir() + m_inputs_filename) + std::string(".stktas...")).c_str());
	std::string line;
	while(std::getline(file, line)) {
		TasInput input;
		if (input.parse(line))
			m_file_inputs.push_back(input);
		else break;
	}

	m_inputs = m_file_inputs;
	Log::info("TAS", (std::string("Inputs for ") + std::to_string(m_inputs.size()) + std::string(" Ticks loaded")).c_str());
	return true;
}

bool Tas::updateInputs() {
	if (m_bfname == "")
		return loadInputs(m_inputs_filename);

	m_inputs = m_file_inputs;
	if (m_bfname == "volcanoJump1" || m_bfname == "volcanoJump2") {
		for (int i(0) ; i < m_bfv[1] ; i++) {
			TasInput steerLeft;
			steerLeft.a = true;
			steerLeft.l = true;
			m_inputs.push_back(steerLeft);
		}
		for (int i(0) ; i < m_bfv[0] - m_bfv[1] - m_bfv[2] ; i++) {
			TasInput accelerate;
			accelerate.a = true;
			m_inputs.push_back(accelerate);
		}
		for (int i(0) ; i < m_bfv[2] ; i++) {
			TasInput steerRight;
			steerRight.a = true;
			steerRight.r = true;
			m_inputs.push_back(steerRight);
		}
		for (int i(0) ; i < 1 ; i++) {
			TasInput skidLeft;
			skidLeft.a = true;
			skidLeft.l = true;
			skidLeft.s = true;
			m_inputs.push_back(skidLeft);
		}
		for (int i(0) ; i < 270 ; i++) {
			TasInput skidLeft;
			skidLeft.a = true;
			skidLeft.l = true;
			skidLeft.s = true;
			m_inputs.push_back(skidLeft);
		}
		for (int i(0) ; i < 180 ; i++) {
			TasInput goLeft;
			goLeft.a = true;
			goLeft.l = true;
			m_inputs.push_back(goLeft);
		}
		
		m_bfv[2]++;
		if (m_bfv[2] + m_bfv[1] > m_bfv[0]) {
			m_bfv[2] = 0;
			m_bfv[1]++;
			Log::warn("TAS", (std::string("alr = ") + std::to_string(m_bfv[0]) + std::string(" ") + std::to_string(m_bfv[1]) + std::string(" ") + std::to_string(m_bfv[2])).c_str());
		}
		if (m_bfv[1] > m_bfv[0]) {
			m_bfv[1] = 0;
			m_bfv[0]++;
		}
		m_checkpoint = m_file_inputs.size() + (m_bfname == "volcanoJump1" ? 505 : 535);
	}
	else
		Log::warn("TAS", "Unknown Brute Force!");
	return true;
}

void Tas::update(const double x, const double y, const double z, const double v) {
	m_prevx = m_x;
	m_prevy = m_y;
	m_prevz = m_z;
	m_prevv = m_v;
	
	m_x = x;
	m_y = y;
	m_z = z;
	m_v = v;
	
	if (m_bfname == "volcanoJump1" || m_bfname == "volcanoJump2") {
		if (m_current_tick > m_file_inputs.size() + (m_bfname == "volcanoJump1" ? 55 : 85) &&
		    m_current_tick < m_file_inputs.size() + (m_bfname == "volcanoJump1" ? 130 : 160) &&
		    m_v - m_prevv < -0.1) { // Turns too far right just after Skid.
			m_bfv[2] = m_bfv[0];
			updateInputs();
			requestReturnToCheckpoint();
			return;
		}
		if (m_current_tick > m_file_inputs.size() + (m_bfname == "volcanoJump1" ? 100 : 130) &&
		    m_current_tick < m_file_inputs.size() + (m_bfname == "volcanoJump1" ? 135 : 165) &&
		    m_v - m_prevv < -0.5) { // Turns too far Left just after Skid, detectable if hit the Banana.
			m_bfv[2] = m_bfv[0];
			m_bfv[1] = m_bfv[0];
			updateInputs();
			requestReturnToCheckpoint();
			return;
		}
	}
	
	if (!m_is_enabled) return;
	if (m_is_paused) {
		if (isBruteForcing()) unpauseReplay();
		else return;
	}
	TasInput input(getCurrentInput()), previousInput;
	if (m_current_tick > 0 && m_current_tick <= m_inputs.size())
		previousInput = m_inputs[m_current_tick - 1];
	
	// Simulate Key Down Events with the Else Ifs.
	if (input.a)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_UP, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.a)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_UP, Input::AD_POSITIVE, 0, false);
	if (input.d)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_DOWN, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.d)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_DOWN, Input::AD_POSITIVE, 0, false);
	if (input.l)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_LEFT, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.l)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_LEFT, Input::AD_POSITIVE, 0, false);
	if (input.r)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_RIGHT, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.r)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_RIGHT, Input::AD_POSITIVE, 0, false);
	if (input.s)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_V, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.s)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_V, Input::AD_POSITIVE, 0, false);
	if (input.n)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_N, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.n)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_N, Input::AD_POSITIVE, 0, false);
	if (input.f)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_SPACE, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.f)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_SPACE, Input::AD_POSITIVE, 0, false);
	if (input.b)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_B, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.b)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_B, Input::AD_POSITIVE, 0, false);
	if (input.t)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_BACK, Input::AD_POSITIVE, Input::MAX_VALUE, false);
	else if (previousInput.t)
		input_manager->dispatchInput(Input::IT_KEYBOARD, 0, IRR_KEY_BACK, Input::AD_POSITIVE, 0, false);

	m_current_tick++;
}

void Tas::setCheckpoint() {
	if (!m_is_enabled) return;
	m_checkpoint = m_current_tick;
	Log::info("TAS", (std::string("Checkpoint now at Tick ") + std::to_string(m_checkpoint)).c_str());
}

void Tas::requestReturnToCheckpoint() {
	if (m_is_enabled)
		Log::info("TAS", (std::string("Returning to Checkpoint at Tick ") + std::to_string(m_checkpoint)).c_str());
	else
		Log::info("Speedrun", "Instant Race Reset.");
	m_restart_requested = true;
}

void Tas::restartRequestProcessed() {
	updateInputs();
	m_current_tick = 0;
	m_restart_requested = false;
}

void Tas::doBruteForceFor(const std::string &bfname) {
	m_bfname = bfname;
	if (m_bfname == "volcanoJump1" || m_bfname == "volcanoJump2") {
		// Brute Forcing the Bounce on Rock to Skip a portion of the Track.
		// Starts just after the Skid ending after the Big Nitro.
		m_bfv[0] = m_bfname == "volcanoJump1" ? 155 : 185; // How many Ticks before starting the next Skid
		m_bfv[1] = 0; // How many Steer Left Ticks just after the previous Skid
		m_bfv[2] = 0; // How many Steer Right Ticks just before the next Skid
	}
	else if (m_bfname != "")
		Log::warn("TAS", "Unknown Brute Force!");
	loadInputs(m_inputs_filename);
	updateInputs();
}

bool Tas::isCandidate() const {
	return false;
}
