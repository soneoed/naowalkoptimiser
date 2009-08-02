#include <iostream>
#include <cstring>
#include "GameController.h"
#include "../Parse.h"
#include "../Locomotion/Walk/sensors.h"
#include "../Behaviour/Leds.h"

//#define GC_DEBUG

using namespace std;

// get_msec: Get the current time in milliseconds.
static long long get_msec()
{
	struct timeval time;
	gettimeofday(&time, NULL);
	return time.tv_sec * 1000 + (time.tv_usec / 1000);
}

//______________________________________________
// constructor
//______________________________________________
GameController::GameController()
{

// Get player values.
	loadTeamCfg();

    returnPacketToBeSent = false;

// Set bumpers to unpressed.
	leftFootLeftBumperPressed = false; 
	leftFootRightBumperPressed = false; 
	rightFootLeftBumperPressed = false; 
	rightFootRightBumperPressed = false;
    chestButtonPressed = false;
    chestHasTriggered = false;

// initialise the chest button timer.
	chestButtonPressTime = get_msec();
	initGameData();
    previousState = controlData.state;
//	writeToALMem(&controlData);
  displayLEDs();	// Set Leds to display the robots current game state.();

#ifdef GC_DEBUG
//	log->debug("GameController","GC Module costructor complete.");
#endif
	return;
}

//______________________________________________
// destructor
//______________________________________________
GameController::~GameController()
{
	return;
}



// Read config information from the specified file.
void GameController::loadTeamCfg(std::string filename)
{
    Parse teamConfiguration;
    playerNumber = -1;
    teamNumber = -1;
    cout << "GameController: Reading Data From " << filename << endl; 
    teamConfiguration.ParseFile("/home/root/player.cfg");
    playerNumber = teamConfiguration.GetAsInt("ROBOT_NUMBER");
    teamNumber = teamConfiguration.GetAsInt("TEAM_NUMBER");
    if((playerNumber == -1) || (teamNumber == -1))
    {
        cout << "!!! WARNING: Player Data Not Loaded !!!" << endl;
        playerNumber = 2;
        teamNumber = 13;
    }
    cout << "GameController: I am player number " << playerNumber << " on team " << teamNumber << endl;
}



/* create the default RoboCupGameControlData, defaulting to blue team */
void GameController::initGameData()
{
    
    memcpy(controlData.header, GAMECONTROLLER_STRUCT_HEADER, sizeof(controlData.header));
    controlData.version       = GAMECONTROLLER_STRUCT_VERSION;

    controlData.playersPerTeam = 4;
    if (playerNumber > 4)
    	controlData.playersPerTeam = playerNumber;

    controlData.state         = STATE_INITIAL;
    controlData.firstHalf     = 1;
    controlData.kickOffTeam   = TEAM_BLUE;
    controlData.secondaryState= STATE2_NORMAL;
    controlData.dropInTeam    = 0;
    controlData.dropInTime    = (uint16)-1;
    controlData.secsRemaining = 0;
    
    TeamInfo* blueTeam = &(controlData.teams[TEAM_BLUE]);
    TeamInfo* redTeam  = &(controlData.teams[TEAM_RED]);
    
    blueTeam->teamColour = TEAM_BLUE;
    blueTeam->teamNumber = teamNumber;
    blueTeam->score      = 0;    
    redTeam->teamColour  = TEAM_RED;
    redTeam->teamNumber  = (uint8)-1;        // undefined team
    redTeam->score       = 0;    
    
    for (int team=0; team<1; team++) {
        for (int player=0; player<MAX_NUM_PLAYERS; player++) {
            controlData.teams[team].players[player].penalty = PENALTY_NONE;
            controlData.teams[team].players[player].secsTillUnpenalised = 0;
        }
    }
    myTeam = blueTeam;
}



// Process an incoming game controller packet. 
void GameController::processPacket(RoboCupGameControlData *packet)
{
    // process only if the packet is good
    if (validatePacket(packet) == false) return;
    previousState = controlData.state;
    bool previouslyPenalised = isPenalised();
    int prevColour = getTeamColour();
        
    // Normalise the team structure order so that BLUE is always first
    if (packet->teams[TEAM_BLUE].teamColour != TEAM_BLUE)
        rawSwapTeams(packet);
        
    // Beep if we just got a wireless command that has us penalized with remaining time 0
    //beepOnUnfairPenalize(packet); // TODO: Make a sound for this.
        
    // print it out if this packet is different to the previous one
    //         and is relevant to this game
    if (!dataEqual(packet, &controlData)) {
        memcpy(&controlData, packet, sizeof(RoboCupGameControlData));
            
        // make myTeam point to the my actual team, based on teamNumber
        if (controlData.teams[TEAM_BLUE].teamNumber == teamNumber) {
            myTeam = &controlData.teams[TEAM_BLUE];
        } else if (controlData.teams[TEAM_RED].teamNumber == teamNumber) {
            myTeam = &controlData.teams[TEAM_RED];
        }

        if(prevColour != getTeamColour()){
            announceTeamColour(getTeamColour());
        }

        if(previousState != controlData.state || previouslyPenalised != isPenalised()){
            announceState(controlData.state);
        }
        displayLEDs();	// Set Leds to display the robots current game state.;
    }      
}

bool GameController::checkForManualInput(){
    returnPacketToBeSent = false;
    if(controlData.state == STATE_READY) return returnPacketToBeSent;
    if(controlData.state == STATE_SET) return returnPacketToBeSent;
    if(controlData.state == STATE_FINISHED) return returnPacketToBeSent;
    bool chest = (touchValues[T_CHEST_BUTTON] > 0.0f);
    

    // Check if the chest button has been pressed.    
    if(chest == true){
        if(chestButtonPressed == false){
            chestButtonPressTime = get_msec();
            chestHasTriggered = false;
        } else if ( ((get_msec()-chestButtonPressTime) > ChestButtonTriggerTime) && (chestHasTriggered == false) ){
            doManualStateChange();
            chestHasTriggered = true;
        }
    }
    chestButtonPressed = chest;

    if(controlData.state != STATE_INITIAL){
        displayLEDs();	// Set Leds to display the robots current game state.
        return returnPacketToBeSent; // Only initial needs to check the foot bumpers.
    }

    // Now check the foot bumpers.
    bool llBump = (touchValues[T_L_BUMP_L] > 0.0f);
    bool lrBump = (touchValues[T_L_BUMP_R] > 0.0f);
    bool rlBump = (touchValues[T_R_BUMP_L] > 0.0f);
    bool rrBump = (touchValues[T_R_BUMP_R] > 0.0f);

    bool leftBumperTrigger = false;
    bool rightBumperTrigger = false;

    if( (leftFootLeftBumperPressed == false) && (leftFootRightBumperPressed == false) ){
        if( (llBump == true) || (lrBump == true) ){
            leftBumperTrigger = true;
        }
    }

    if( (rightFootLeftBumperPressed == false) && (rightFootRightBumperPressed == false) ){
        if( (rlBump == true) || (rrBump == true) ){
            rightBumperTrigger = true;
        }
    }

    leftFootLeftBumperPressed = llBump;
    leftFootRightBumperPressed = lrBump; 
    rightFootLeftBumperPressed = rlBump;
    rightFootRightBumperPressed = rrBump; 
        
    if( leftBumperTrigger == true ){
        // Swap teams
		swapTeams();
        controlData.kickOffTeam = !controlData.kickOffTeam; // Preserve if you are kicking off or not.
    }
    
    if( rightBumperTrigger == true ){
        // Swap Kickoff
		controlData.kickOffTeam = !controlData.kickOffTeam;
    }
    displayLEDs();	// Set Leds to display the robots current game state.
    return returnPacketToBeSent;
}

bool GameController::isPlayerPenalised(int playerNumber, int teamColour)
{
	bool penalised = (controlData.teams[teamColour].players[playerNumber-1].penalty != PENALTY_NONE) && (controlData.state == STATE_PLAYING);
    return penalised;
}

// Swap the team colours. The teams are physically moved in memory to preserve the
// array index corresponding to the colour.
void GameController::swapTeams()
{
    int currColour = myTeam->teamColour; // Get 
    rawSwapTeams(&controlData);
	// Swap colours
	controlData.teams[TEAM_BLUE].teamColour = TEAM_BLUE; 
	controlData.teams[TEAM_RED].teamColour = TEAM_RED; 
	if(currColour == TEAM_BLUE){
		myTeam = &(controlData.teams[TEAM_RED]);
	} else if (currColour == TEAM_RED){
		myTeam = &(controlData.teams[TEAM_BLUE]);
	}
    announceTeamColour(getTeamColour());
	return;
}

void GameController::rawSwapTeams(RoboCupGameControlData* data) {
    size_t    teamSize = sizeof(TeamInfo);
    TeamInfo* blueTeam = &(data->teams[TEAM_BLUE]);
    TeamInfo* redTeam  = &(data->teams[TEAM_RED]);
    
    TeamInfo tempTeam;
    memcpy(&tempTeam, blueTeam, teamSize);    
    
    // swap the teams
    memcpy(blueTeam, redTeam, teamSize);
    memcpy(redTeam, &tempTeam, teamSize);
}

/* runs various checks and returns true/false for good/bad packet */
bool GameController::validatePacket(RoboCupGameControlData *data) {
        
    // check the right structure header has come in
    if (!(checkHeader(data->header))) {
//			log->debug("GameController", "Data Header Mismatch.");      
      return false;
    }
    
    // check for partial packets
    if (sizeof(*data) != sizeof(RoboCupGameControlData)) {
//      log->debug("GameController", "Received Partial Packet.");
      return false;
    }    
        
    // check the right version of the structure is being used
    if (data->version != GAMECONTROLLER_STRUCT_VERSION) {
//        log->debug("GameController", "Data Version Mismatch.");
        return false;                 
    }   

    // check whether this packet belongs to this game at all
    if (!isThisGame(data)) return false;
     
    return true;
}



// checks the header of a packet. Since the GameController broadcasts 4 
//   characters and not a string (which is terminated by a \0), we should check
//   each character individually instead of using something like strcmp
bool GameController::checkHeader(char* header) {
    for (int i=0; i<4; i++)
        if (header[i] != GAMECONTROLLER_STRUCT_HEADER[i]) return false;
    return true;    
}



// compare two byte streams, returns true (match) or false (no match)
bool GameController::dataEqual(void* data, void* previous) {
    if (!memcmp(previous, data, sizeof(RoboCupGameControlData)))
        return true;
    return false;
}


// checks whether a packet even belongs in this game
bool GameController::isThisGame(RoboCupGameControlData* gameData) {
 
    if (gameData->teams[TEAM_BLUE].teamNumber != teamNumber &&
        gameData->teams[TEAM_RED].teamNumber  != teamNumber) {
        return false;
    }   
    return true;
}



// Trigger a manual state change.
int GameController::doManualStateChange()
{
	if( isPenalised() == true ){
        // If the robot is penalised, unpenalise it.
		manualPenalise(false);
	}
    else {
	    switch(controlData.state)
	    {
		    case STATE_INITIAL:
			    controlData.state = STATE_PLAYING;
			    break;
		    case STATE_PLAYING:
			    manualPenalise(true);
			    break;
		    default:
			    break;
	    }
    }
    announceState(controlData.state);
	return controlData.state;
}

// apply manual penalise to the robot.  The bool allows
// both penalise and unpenalise in this function.
void GameController::manualPenalise(bool penalise) {
    myTeam->players[playerNumber-1].penalty = penalise?PENALTY_MANUAL:PENALTY_NONE;
    
    //
    // Broadcast a message so the GameController knows we're penalised
    //
    memcpy(returnPacket.header, GAMECONTROLLER_RETURN_STRUCT_HEADER, sizeof(returnPacket.header));
    returnPacket.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
    returnPacket.team = teamNumber;
    returnPacket.player = playerNumber;
    returnPacket.message = penalise?GAMECONTROLLER_RETURN_MSG_MAN_PENALISE:
                                    GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE;

    // Set flag to send the packet
    returnPacketToBeSent = true;
    return;
}


void GameController::announceState(int newState)
{
    if(isPenalised() == true){
        cout << "State: Penalised." << endl;
        balanceFallingEnabled = false;
        system("aplay /home/root/SoundStates/penalty.wav");
    }
    else {
        switch(newState){
            case STATE_INITIAL:
                cout << "State: Initial." << endl;
                balanceFallingEnabled = false;
                system("aplay /home/root/SoundStates/initial.wav");
                break;

            case STATE_READY:
                cout << "State: Ready." << endl;
                balanceFallingEnabled = true;
                system("aplay /home/root/SoundStates/ready.wav");
                break;

            case STATE_SET:
                cout << "State: Set." << endl;
                balanceFallingEnabled = false;
                system("aplay /home/root/SoundStates/set.wav");
                break;
            
            case STATE_PLAYING:
                cout << "State: Playing." << endl;
                balanceFallingEnabled = true;
                system("aplay /home/root/SoundStates/playing.wav");
                break;

            case STATE_FINISHED:
                cout << "State: Finished." << endl;
                balanceFallingEnabled = false;
                system("aplay /home/root/SoundStates/finnish.wav");
                break;      

            default:
                cout << "State: Unknown!" << endl;
        }
    }
    return;
}


void GameController::announceTeamColour(int teamColour)
{
  switch(teamColour){
    case TEAM_BLUE:
      cout << "Team: Blue." << endl;
      system("aplay /home/root/SoundStates/blue.wav");
      break;
    case TEAM_RED:
      cout << "Team: Red." << endl;
      system("aplay /home/root/SoundStates/red.wav");
      break;
    default:
      cout << "Team: Unknown!" << endl;
  }
}

// Refreshes the LEDs with current state.
void GameController::displayLEDs()
{
	bool penalised = (myTeam->players[playerNumber-1].penalty != PENALTY_NONE) && (controlData.state == STATE_PLAYING);
	if(penalised)
	{
		// Set chest RED
		Leds::setChest(1.0,0.0,0.0);
	}
	else 
	{
		switch(controlData.state)
		{
			case STATE_INITIAL:
				// Set chest OFF.
				Leds::setChest(0.0,0.0,0.0);
				break;
			case STATE_READY:
				// Set chest BLUE
				Leds::setChest(0.0,0.0,1.0);
				break;
			case STATE_SET:
				// Set chest YELLOW
				Leds::setChest(0.4,1.0,0.0);
				break;
			case STATE_PLAYING:
				// Set chest GREEN
				Leds::setChest(0.0,1.0,0.0);
				break;
			case STATE_FINISHED:
				// Set chest OFF
				Leds::setChest(0.0,0.0,0.0);
			default:
				break;
		}
	}
	
	if(myTeam->teamColour == TEAM_BLUE)
	{
        // Set Left foot BLUE
		Leds::setLeftFoot(0.0,0.0,1.0);
	} else {
        // Set Left foot RED
		Leds::setLeftFoot(1.0,0.0,0.0);
	}

	switch(controlData.state){
		case STATE_INITIAL:
		case STATE_READY:
		case STATE_SET:
			if( isKickingOff() )	
			{
                // Set Right foot YELLOW
				Leds::setRightFoot(0.4,1.0,0.0);
			} else {
                // Set Right foot OFF
				Leds::setRightFoot(0.0,0.0,0.0);
			}
			break;
		default:
            // Set Right foot OFF      
			Leds::setRightFoot(0.0,0.0,0.0);
			break;
	}
	return;
}
