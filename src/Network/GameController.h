// NUbots Game Controller

#ifndef GameController_H
#define GameController_H
#include "../Globals.h"
#include "RoboCupGameControlData.h"


class GameController
{

    public:

        static GameController& getInstance()
        {
          static GameController gameControllerInstance;
          return gameControllerInstance;
        }

	    void processPacket(RoboCupGameControlData *packet);	// process the incoming gamecontrollere packet.
        bool checkForManualInput();

        // Access game info
        int getTeamNumber() { return teamNumber; };
        int getPlayerNumber() { return playerNumber; };

        int getGameState() { return controlData.state; };
        int getSecondaryState() { return controlData.secondaryState; };

        int getTeamColour(){ return myTeam->teamColour; };
        int getKickoffTeam() { return controlData.kickOffTeam; };
        bool isKickingOff() { return ( getKickoffTeam() == getTeamColour() );};
        bool isFirstHalf() { return (bool)controlData.firstHalf; };
        bool isPenaltyShootout() { return (controlData.secondaryState == STATE2_PENALTYSHOOT); };
        int getRemainingGameTime() { return controlData.secsRemaining; };

        int getTimeSinceLastBallOut() { return controlData.dropInTime; };
        int getLastBallOutTeam() { return controlData.dropInTeam; };
        bool isPenalised(){ return isPlayerPenalised(getPlayerNumber(), getTeamColour()); };
        bool isPlayerPenalised(int playerNumber, int teamColour);
        int getRemainingPenaltyTime(){ return getPlayerPenaltyTimeRemaining(getPlayerNumber(), getTeamColour());};
        int getPlayerPenaltyTimeRemaining(int player, int teamColour);
        int getTeamScore(int teamColour);
        
        bool getReturnPacketToBeSent() { return returnPacketToBeSent; };
        RoboCupGameControlReturnData getReturnPacket() { returnPacketToBeSent = false; return returnPacket; };

        RoboCupGameControlData getControlData();
    

    private:

        // Sensor Tracking Variables
	    bool leftFootLeftBumperPressed, leftFootRightBumperPressed, rightFootLeftBumperPressed, rightFootRightBumperPressed; // Storage of foot bumper states
        bool chestButtonPressed;
        bool chestHasTriggered;
	    static const int ChestButtonTriggerTime = 150; // Button press threshold in milliseconds.
	    long long chestButtonPressTime; // Time Keeping for chest button

        // Game information
        RoboCupGameControlData controlData; // Storage of the current game controller data.
	    int playerNumber; // The robots player number.
	    int teamNumber; // The robots team number.
	    TeamInfo* myTeam; // Pointer to the game data specific to the robots current team. 
        int previousState;
        RoboCupGameControlReturnData returnPacket;
        bool returnPacketToBeSent;

        // Init Functions
	    void initGameData();	// Setup default values for gameData.
	    void loadTeamCfg(std::string filename = "/home/root/player.cfg");	// Read in configuration data from file.

        // Sounds
        void announceState(int state);
        void announceTeamColour(int teamColour);
        void announceKickoff(bool kickingOff);
        void playUnfairPenaltySound();

        // Others
	    void displayLEDs();	// Set Leds to display the robots current game state.
	    void swapTeams();	// Swaps the colours of the teams.
        void rawSwapTeams(RoboCupGameControlData* data);
	    bool validatePacket(RoboCupGameControlData* packet);	// Checks that the gamecontroller packet is of the right version and for the correct game.
        bool isThisGame(RoboCupGameControlData* data);
        bool dataEqual(void* data, void* prev);
        bool checkHeader(char* header);
	    int doManualStateChange();	// Called when a state change has been manually triggered by the chest button.
	    void manualPenalise(bool penalty);	// Called when a penalty change is triggered by the chest button.

    private:
        // Hidden for static class
        GameController();                             // Private constructor
        ~GameController();                             // Private destructor
        GameController(const GameController&);            // Prevent copy constructor
        GameController& operator=(const GameController&); // Prevent assignment

};
#endif // GameController_H
