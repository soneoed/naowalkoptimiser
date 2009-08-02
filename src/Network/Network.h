#ifndef NETWORK_H_DEFINED
#define NETWORK_H_DEFINED

#include "../Globals.h"
#include "../Locomotion/Walk/sensors.h"
#include "RoboCupGameControlData.h"
#include "UdpPort.h"
#include "../NUbotImage.h"



class Network
{
	public:
		Network();
		~Network();
		void ProcessUpdates();
		void SendUpdate();
        
	private:
        //NETWORK INTERFACE
        #define STOPSTREAM '0'        
        #define STARTSTREAM '1'
        #define SENDPACKET '2'        
	    //#define GAME_CONTROLLER_PORT 3838
        UdpPort *udp_gc;
	   /* int sockfd;
	    struct sockaddr_in my_addr;    // my address information
	    struct sockaddr_in their_addr; // connector's address information
	    socklen_t addr_len;
	    int numbytes;
	    fd_set master;   // master file descriptor list
	    fd_set read_fds; // temp file descriptor list for select()
	    int fdmax; */
	    RoboCupGameControlData *GControlData;
	    NUbotImage* imageClass;
        //LOCWM Variables
        
        #define WALK_PORT 6767
        #define LOCWM_PORT 7878
        #define VISION_PORT 8532
        #define CLASSIFIED_PORT 8533
        #define GAMEPACKET_PORT 14538
        UdpPort *udp_walk;
        UdpPort *udp_locWM;
        UdpPort *udp_Vision;
        UdpPort *udp_Classified;
        UdpPort *udp_GamePacket;
        bool locWMStream;
        bool classStream;
        bool visionStream;
        void CheckGameController();
        void CheckGamePacket();
        void CheckWalkPacket();
        void SendLocWMPacket();
        void SendVisionPacket();
        void SendClassifiedPacket();
        void SendGamePacket();
        int ConstructNewWMDataPacket(byte*);
        int ConstructOdometryPacketExtended(byte* );
        int ConstructWMExtraPacket(byte*);
        int ConstructOdometryPacket(byte*);
        int ConstructNewVisionPacket(byte*);
        int ConstructNewClassifiedImagePacket(byte*);
        int ConstructTeamPacket(byte* data);
    /*    int locWMSockfd;
        struct sockaddr_in locWM_my_addr;    // my address information
        struct sockaddr_in locWM_their_addr;    // my address information
   	    fd_set master;   // master file descriptor list
	    fd_set read_fds; // temp file descriptor list for select()
	    int fdmax;
      */  
};

#endif
