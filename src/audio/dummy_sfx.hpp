//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2006-2015 Patrick Ammann <pammann@aro.ch>
//  Copyright (C) 2008-2015 Joerg Henrichs, Patrick Ammann
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

#ifndef HEADER_DUMMY_SFX_HPP
#define HEADER_DUMMY_SFX_HPP

#include "audio/sfx_base.hpp"

/**
 * \brief Dummy sound when ogg or openal aren't available
 * \ingroup audio
 */
class DummySFX : public SFXBase
{
public:
                       DummySFX(SFXBuffer* buffer, bool positional,
                                float gain) {}
    virtual           ~DummySFX() {}

    /** Late creation, if SFX was initially disabled */
    virtual bool       init() override { return true;  }
    virtual bool       isLooped() override { return false; }
    virtual void       updatePlayingSFX(float dt) override {}
    virtual void       setLoop(bool status) override {}
    virtual void       reallySetLoop(bool status) override {}
    virtual void       setPosition(const Vec3 &p) override {}
    virtual void       reallySetPosition(const Vec3 &p) override {}
    virtual void       setSpeedPosition(float factor,
                                        const Vec3 &p) override {}
    virtual void       reallySetSpeedPosition(float f,
                                         const Vec3 &p) override {}
    virtual void       play() override {}
    virtual void       reallyPlayNow(SFXBuffer* buffer = NULL) override {}
    virtual void       play(const Vec3 &xyz, SFXBuffer* buffer = NULL) override {}
    virtual void       reallyPlayNow(const Vec3 &xyz, SFXBuffer* buffer = NULL) override {}
    virtual void       stop() override {}
    virtual void       reallyStopNow() override {}
    virtual void       pause() override {}
    virtual void       reallyPauseNow() override {}
    virtual void       resume() override {}
    virtual void       reallyResumeNow() override {}
    virtual void       deleteSFX() override {}
    virtual void       setSpeed(float factor) override {}
    virtual void       reallySetSpeed(float factor) override {}
    virtual void       setVolume(float gain) override {}
    virtual void       reallySetVolume(float gain) override {}
    virtual void       setMasterVolume(float gain) override {}
    virtual void       reallySetMasterVolumeNow(float gain) override {}
    virtual SFXStatus  getStatus() override { return SFX_STOPPED; }
    virtual void       onSoundEnabledBack(bool resume_later) override {}
    virtual void       setRolloff(float rolloff) override {}
    virtual SFXBuffer* getBuffer() const override { return NULL; }

};   // DummySFX


#endif // HEADER_SFX_HPP

