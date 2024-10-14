//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2019 Fouks
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

#include <fstream>
#include <sstream>

#include "graphics/irr_driver.hpp"
#include "guiengine/message_queue.hpp"
#include "karts/abstract_kart.hpp"
#include "karts/controller/player_controller.hpp"
#include "modes/linear_world.hpp"
#include "tas.hpp"
#include "tracks/track.hpp"
#include "utils/log.hpp"

std::string TasInput::toString() const
{
    std::ostringstream oss;
    oss << (uint16_t) m_action << " " << m_steer << " " << m_accel;
    return oss.str();
}

bool TasInput::parse(std::string line)
{
    int32_t data[3];
    if (sscanf(line.c_str(), "%d %d %d\n", &data[0], &data[1], &data[2]) != 3)
    {
        Log::error("TAS", "Error parsing input %s !", line.c_str());
        *this = TasInput();
        return false;
    }
    m_action = (uint8_t) data[0];
    m_steer  = data[1];
    m_accel  = (uint16_t) data[2];
    return true;
}

Tas *Tas::m_tas = NULL;

Tas::Tas()
{
    init();
}

Tas::~Tas()
{

}

void Tas::reset()
{
    m_game_status = GameStatus::NORMAL;
    m_inited_for_race = false;
    m_current_tick = 0;
    m_world = NULL;
    m_player_kart = NULL;
    m_is_recording_frames = false;
    m_frame_number = 0;
    m_is_recording_inputs = false;
    m_recorded_inputs.clear();
    m_has_started = false;
    m_inputs_to_play_filename = "";
    m_inputs_to_play.clear();
    m_save_state.reset();
}

void Tas::resetForRace()
{
    std::string inputs_to_play_filename = m_inputs_to_play_filename;
    reset();
    m_inputs_to_play_filename = inputs_to_play_filename;
}

void Tas::init()
{
    reset();
    Log::info("TAS", "Inited.");
}

void Tas::initForRace(bool record_inputs)
{
    resetForRace();
    LinearWorld* lw = dynamic_cast<LinearWorld*>(World::getWorld());
    if (!lw)
    {
        Log::warn("TAS", "Cannot init for race here!");
        return;
    }

    // Find the Player Kart
    for(unsigned int i = 0 ; i < RaceManager::get()->getNumberOfKarts() ; i++)
    {
        AbstractKart *kart = lw->getKart(i);
        PlayerController *pc = dynamic_cast<PlayerController*>(kart->getController());
        if (pc)
        {
            m_player_kart = kart;
            break;
        }
    }
    if (!m_player_kart)
    {
        Log::warn("TAS", "There are no player kart here!");
        return;
    }

    m_is_recording_inputs = record_inputs;
    if (m_is_recording_inputs) Log::info("TAS", "Inputs will be recorded.");

    if (m_inputs_to_play_filename != "")
    {
        if (loadInputs()) Log::info("TAS", "Inputs successfully loaded.");
    }
    m_world = lw;
    m_inited_for_race = true;
    Log::info("TAS", "Successfully inited for the race.");
}

void Tas::saveFrame() {
    irr_driver->doFrameShot(m_frame_number);
    m_frame_number++;
}
void Tas::startRecordingFrames() {
    m_frame_number = 0;
    m_is_recording_frames = true;
}
void Tas::stopRecordingFrames() {
    m_is_recording_frames = false;
    m_frame_number = 0;
}

void Tas::update()
{
    if (!m_inited_for_race) return;
    PlayerController *pc = dynamic_cast<PlayerController*>(m_player_kart->getController());
    if (m_is_recording_inputs)
    {
        if ((m_world->getPhase() == World::RESULT_DISPLAY_PHASE) || !pc) // Race ended
        {
            m_is_recording_inputs = false;
            saveInputs();
        }
        else
            m_recorded_inputs.push_back(TasInput(m_player_kart->getControls().getButtonsCompressed(), pc->m_steer_val, m_player_kart->getControls().m_accel));
    }
    m_current_tick++;
}

unsigned int Tas::getCurrentKartId() const
{
    if (!m_player_kart) return -1;
    return m_player_kart->getWorldKartId();
}

void Tas::saveInputs()
{
    if (!m_inited_for_race) return;
    float min_time = m_player_kart->getFinishTime();

    int day, month, year;
    StkTime::getDate(&day, &month, &year);
    std::string time = std::to_string(min_time);
    std::replace(time.begin(), time.end(), '.', '_');
    std::ostringstream oss;
    oss << Track::getCurrentTrack()->getIdent() << "_" << year << month << day << "_" << m_world->getNumKarts() << "_" << time << ".stktas";

    std::ofstream file(file_manager->getReplayDir() + oss.str());
    if (!file)
    {
        Log::error("TAS", "Cannot open '%s' to save Inputs!", oss.str().c_str());
        return;
    }
    for (unsigned int i = 0; i < m_recorded_inputs.size(); i++)
        file << m_recorded_inputs[i].toString() << std::endl;
    file.close();

    Log::info("TAS", (std::string("Successfully saved inputs in ") + file_manager->getReplayDir() + oss.str()).c_str());
}

bool Tas::loadInputs()
{
    m_inputs_to_play.clear();
    std::ifstream file(file_manager->getReplayDir() + m_inputs_to_play_filename + ".stktas");
    if (!file)
    {
        Log::info("TAS", (std::string("Inputs file ") + std::string(file_manager->getReplayDir() + m_inputs_to_play_filename) + std::string(".stktas not found, no inputs replay")).c_str());
        return false;
    }

    Log::info("TAS", (std::string("Inputs file ") + std::string(file_manager->getReplayDir() + m_inputs_to_play_filename) + std::string(".stktas loaded")).c_str());
    std::string line;
    while(std::getline(file, line))
    {
        TasInput input;
        if (input.parse(line)) m_inputs_to_play.push_back(input);
        else break;
    }
    Log::info("TAS", (std::string("Inputs for ") + std::to_string(m_inputs_to_play.size()) + std::string(" ticks loaded")).c_str());
    file.close();
    return true;
}

bool Tas::isInputReplayFinished() const
{
    if (!m_inited_for_race) return true;
    if (!m_world) return true;
    if (!m_player_kart) return true;
    if (m_world->getPhase() == World::RESULT_DISPLAY_PHASE) return true;
    if (!dynamic_cast<PlayerController*>(m_player_kart->getController())) return true;
    return m_current_tick >= m_inputs_to_play.size();
}

void Tas::applyCurrentInput()
{
    if (isInputReplayFinished()) return;
    PlayerController *pc = dynamic_cast<PlayerController*>(m_player_kart->getController());
    TasInput input = getCurrentInput();
    m_player_kart->getControls().setButtonsCompressed(input.getAction());
    pc->m_steer_val = input.getSteer();
    m_player_kart->getControls().m_accel = input.getAccel();
    if (!m_has_started && input.getAccel() > 0) // Fix StartUp Boost
    {
        float f = m_player_kart->getStartupBoostFromStartTicks(m_world->getAuxiliaryTicks());
        m_player_kart->setStartupBoost(f);
        m_has_started = true;
    }
}

TasInput Tas::getCurrentInput() const
{
    if (isInputReplayFinished()) return TasInput();
    else return m_inputs_to_play[m_current_tick];
}

void Tas::saveState()
{
    if (!m_inited_for_race || !m_world || !m_player_kart)
    {
        Log::info("TAS", "Cannot save state here!");
        return;
    }
    if (m_current_tick < 603)
    {
        Log::info("TAS", "Cannot save state before start!");
        return;
    }
    m_save_state.create(m_current_tick, m_world, m_player_kart);
    if (m_save_state.isValid()) Log::info("TAS", (std::string("State saved for tick ") + std::to_string(m_save_state.getTick())).c_str());
}

void Tas::restoreState()
{
    if (!m_save_state.isValid())
    {
        Log::info("TAS", "No valid state to restore!");
        return;
    }
    if (!m_inited_for_race || !m_world || !m_player_kart)
    {
        Log::info("TAS", "Cannot restore state here!");
        return;
    }
    if (m_current_tick < 603)
    {
        Log::info("TAS", "Cannot restore state before start!");
        return;
    }
    m_save_state.restore(m_world, m_player_kart);
    m_current_tick = m_save_state.getTick();
    loadInputs();
    Log::info("TAS", (std::string("Restored state to tick ") + std::to_string(m_current_tick)).c_str());
}
