//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2015 SuperTuxKart-Team
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

#ifndef CUTSCENE_WORLD_HPP
#define CUTSCENE_WORLD_HPP

#include "modes/world_with_rank.hpp"
#include "states_screens/race_gui_base.hpp"

namespace irr
{
    namespace scene { class ICameraSceneNode; }
}


#include <string>

class TrackObject;

/**
 * \brief An implementation of World, to provide animated 3D cutscenes
 * \ingroup modes
 */
class CutsceneWorld : public World
{
    irr::scene::ICameraSceneNode* m_camera;

    std::map<float, std::vector<TrackObject*> > m_sounds_to_trigger;
    std::map<float, std::vector<TrackObject*> > m_sounds_to_stop;
    std::map<float, std::vector<TrackObject*> > m_particles_to_trigger;

    double m_duration;
    bool m_aborted;
    bool m_cleared_cutscene;

    float m_fade_duration;

    // TODO find a better way than static
    static bool s_use_duration;

    /** monkey tricks to get the animations in sync with irrlicht. we reset the time
     *  after all is loaded and it's running withotu delays
     */
    bool m_second_reset;
    double m_time_at_second_reset;

    std::vector<std::string> m_parts;

    void clearCutscene();
public:

    CutsceneWorld();
    virtual ~CutsceneWorld();

    virtual void init() override;

    virtual void reset(bool restart=false) override;

    // clock events
    virtual bool isRaceOver() override;

    virtual const std::string& getIdent() const override;

    virtual void update(int ticks) override;

    virtual void createRaceGUI() override;

    virtual void enterRaceOverState() override;

    // ------------------------------------------------------------------------
    virtual bool raceHasLaps() override { return false; }
    // ------------------------------------------------------------------------
    virtual unsigned int getNumberOfRescuePositions() const override
    {
        return 0;
    }   // getNumberOfRescuePositions
    // ------------------------------------------------------------------------
    virtual unsigned int getRescuePositionIndex(Kart *kart) override
    {
        return 0;
    }   // getRescuePositionIndex
    // ------------------------------------------------------------------------
    virtual btTransform  getRescueTransform(unsigned int index) const override
    {
        return btTransform();
    }   // getRescueTransform
    // ------------------------------------------------------------------------
    virtual void onFirePressed(Controller* who) override { abortCutscene(); }
    // ------------------------------------------------------------------------
    void setParts(std::vector<std::string> parts) { m_parts = parts; }
    // ------------------------------------------------------------------------
    /** Returns the data to display in the race gui.
     */
    virtual void getKartsDisplayInfo(
                  std::vector<RaceGUIBase::KartIconDisplayInfo> *info) override
    {
    };
    // ------------------------------------------------------------------------
    virtual void escapePressed() override { abortCutscene(); }
    // ------------------------------------------------------------------------
    static void setUseDuration(bool use_duration) { s_use_duration = use_duration; }
    // ------------------------------------------------------------------------
    void abortCutscene() { m_aborted = true; }

};   // CutsceneWorld


#endif
