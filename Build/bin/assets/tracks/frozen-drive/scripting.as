/* ---------- MODIFY THIS SECTION IF YOU ARE USING STK 1.0 OR LESS ---------- */

int local_player_kart = -1;

// NOTE: set this to false if you are using STK 1.0 or less!
const bool STK_1_1_OR_GREATER = false;

// NOTE: Will only work from STK 1.1 onwards! Remove the first return line if using < 1.1
bool isLocalPlayer(int kartId) {
//    return (Track::getKartType(kartId) == 0);
    return false;
}

// NOTE: Will only work from STK 1.1 onwards! Remove the first return line if using < 1.1
bool isGhostPlayer(int kartId) {
//    return (Track::getKartType(kartId) == 4);
    return false;
}

/* ---------- END SECTION ---------- */




//TODO: investigate effects on multiplayer, especially SFX-as-music. Does it work in 1.0? 1.1?

//TODO redundant?////
enum icecream_states
{
  MELTED,
  CARAMEL,
  VANILLA,
  STRAWBERRY,
  NONE
};

int icecream = NONE;
array<string> icecreams = {"icecream_melted",
                           "icecream_caramel",
                           "icecream_vanilla",
                           "icecream_strawberry"};

array<int> icecream_times = {0,   // N/A for melted
                             30,  // for caramel
                             22,  // for vanilla
                             16}; // for strawberry

int icecream_seconds;

enum route_states
{
  NORMAL_TRACK,
  TRANSITION,
  DEATH_ROUTE
};

array<int> route_state(Track::getNumberOfKarts(), NORMAL_TRACK);

bool isMultiplayer() {
    return Track::getNumLocalPlayers() > 1;
}

bool isTrackReverse(Track::TrackObject@ obj) {
    return Track::isReverse();
}


void onStart() {
    // terminate any ongoing SFX-as-music and wait until start of race (5 seconds) before starting it again
    // this is a hacky fix to SFX not ending after a restart
    Track::getTrackObject("", "sfx_as_music_frozen").getSoundEmitter().stop();
    Track::getTrackObject("", "sfx_as_music_teddy").getSoundEmitter().stop();
    // in case of restart
    if (local_player_kart >= 0) {
        route_state[local_player_kart] = NORMAL_TRACK;
    }
    icecream = NONE;
    updateIcecreams();
    Utils::setTimeout("playMusic", 5.0);
}

// Should be triggered immediately. This function gets the ID of the local player's kart and records it.
// HACK: This takes advantage of unexpected return value of action trigger callback in STK <= 1.0, which returns the camera's kart, not the kart which triggered the function call.
void recordPlayerKart(int idKart) {
    if (STK_1_1_OR_GREATER){
        // iterate over all karts, checking for the first which is a local player
        int kart = 0;
        while(local_player_kart == -1 && kart < Track::getNumberOfKarts())
        {
            if (isLocalPlayer(kart)) {
                local_player_kart = kart;
            }
            kart++;
        }
        // if none exist, check for a ghost kart
        if (local_player_kart == -1 && isGhostPlayer(0)) {
            local_player_kart = 0;
        }
    } else /* if STK version < 1.1 */ {
        if (!isMultiplayer()) { // multiplayer is currently unsupported
            local_player_kart = idKart;
        }
    }
}

// Calculates whether the local kart is inside a sphere centered at the given trigger, with given radius
// NOTE: this could cause a crash in multiplayer, since local_player_kart == -1
bool isLocalPlayerInTrigger(const string trigger_name, float radius){
    if (local_player_kart == -1) {
        Utils::logWarning(Utils::insertValues("Tried to check if player -1 in is in trigger %s!", trigger_name));
        return false;
    }
    Vec3 loc_kart = Kart::getLocation(local_player_kart);
    Vec3 loc_trigger = Track::getTrackObject("", trigger_name).getCenterPosition();
    if ( 
         (( loc_trigger.getX() - loc_kart.getX() ) * ( loc_trigger.getX() - loc_kart.getX() )) + 
         (( loc_trigger.getY() - loc_kart.getY() ) * ( loc_trigger.getY() - loc_kart.getY() )) +
         (( loc_trigger.getZ() - loc_kart.getZ() ) * ( loc_trigger.getZ() - loc_kart.getZ() ))
         <= (radius * radius) ){
        return true;
    }
    return false;
}

void playMusic()
{
    Track::getTrackObject("", "sfx_as_music_teddy").getSoundEmitter().playLoop();
}

/* Ice-cream state changes */

// Increment the second counter every second until no longer needed. If an ice-cream time expires, change it.
void TimeIcecream() {
   // stop timing a couple of seconds after last state has been reached
   if (icecream != NONE && icecream > MELTED && icecream_seconds < icecream_times[1] + 2) {
      // check if we need to change to another icecream
      if (icecream_seconds >= icecream_times[icecream]) {
          icecream--;
          updateIcecreams();
      }

      icecream_seconds++;
      Utils::setTimeout("TimeIcecream", 1.0);
   }
}

void collectIcecream (int idKart) {
    if (local_player_kart > -1 && isLocalPlayerInTrigger("icecream_trigger", 1.5)) {
        // show message
        if(icecream == MELTED){GUI::displayOverlayMessage("Melted");}
        if(icecream == CARAMEL){GUI::displayOverlayMessage("Caramel");}
        if(icecream == VANILLA){GUI::displayOverlayMessage("Vanilla");}
        if(icecream == STRAWBERRY){GUI::displayOverlayMessage("Strawberry");}

        finishDeathRoute(-1);
    }
}

void updateIcecreams() {
    int i = 0;
    while(i < NONE)
    {
        // if this object is the current icecream flavour, set enabled. Otherwise set false.
        Track::getTrackObject("", icecreams[i]).setEnabled(icecream == i);
        i++;
    }
}


/* Route state changes */

// run when nearing the death route
void transitionIntoDeathRoute(int idKart)
{
    if (local_player_kart > -1 && route_state[local_player_kart] == NORMAL_TRACK
        && isLocalPlayerInTrigger("road_expert_transition", 20.0)){
        Track::getTrackObject("", "sfx_as_music_teddy").getSoundEmitter().stop();
        Track::getTrackObject("", "sfx_death_route_alert").getSoundEmitter().playOnce();
        icecream = NONE;
        route_state[local_player_kart] = TRANSITION;
    }
}

// run when leaving the death route via entrance
void retreatDeathRoute(int idKart)
{
    if (local_player_kart > -1 && route_state[local_player_kart] != NORMAL_TRACK
        && isLocalPlayerInTrigger("road_expert_retreat", 50.0)){
        Track::getTrackObject("", "sfx_as_music_frozen").getSoundEmitter().stop();
        Track::getTrackObject("", "sfx_as_music_teddy").getSoundEmitter().playLoop();
        icecream = NONE;
        route_state[local_player_kart] = NORMAL_TRACK;
    }
}

void delayFrozenLoop() {
    Track::getTrackObject("", "sfx_as_music_frozen").getSoundEmitter().playLoop();
}

void startDeathRoute(int idKart)
{
    if (local_player_kart > -1 && route_state[local_player_kart] == TRANSITION
        && isLocalPlayerInTrigger("road_expert_start", 10.0)){
        Utils::setTimeout("delayFrozenLoop", 0.5);
        icecream = STRAWBERRY;
        updateIcecreams();
        icecream_seconds = 0;
        TimeIcecream();
        route_state[local_player_kart] = DEATH_ROUTE;
    }
}

// run when collecting the ice-cream cone (TODO use trigger to force collection of ice-cream)
void finishDeathRoute(int idKart)
{
    if (local_player_kart > -1 && route_state[local_player_kart] == DEATH_ROUTE
        && (idKart == -1 || isLocalPlayerInTrigger("road_expert_finish", 10.0))){
        // remove any remaining icecream
        icecream = NONE;
        updateIcecreams();

        // change music
        Track::getTrackObject("", "sfx_as_music_frozen").getSoundEmitter().stop();
        
        // play SFX if ice-cream was not collected
        if (idKart != -1) {
            Track::getTrackObject("", "sfx_death_route_alert").getSoundEmitter().playOnce();
        }
        //play Teddy again after 2 seconds
        Utils::setTimeout("playMusic", 2.0);
        route_state[local_player_kart] = NORMAL_TRACK;
    }
}

bool isTimeTrial(Track::TrackObject@ obj) {
    return (Track::getMinorRaceMode() == 1101);
}

void timeTrialWall(int idKart, const string libraryInstance, const string obj_id) {
    if (idKart == local_player_kart) {
        GUI::displayOverlayMessage("Disabled in Time Trial mode");
    }
}
