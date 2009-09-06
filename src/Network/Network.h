#ifndef NETWORK_H_DEFINED
#define NETWORK_H_DEFINED

#include "../Globals.h"
#include "../Locomotion/Walk/sensors.h"
#include "../jwalkoptimiser.h"
#include "UdpPort.h"



class Network
{
	public:
		Network();
		~Network();
		void ProcessUpdates();
		void SendUpdate();
        
	private:   
        
        #define WALK_PORT 6767
        UdpPort *udp_walk;

        void CheckWalkPacket();
    
        void SendOdometryPacket();
        int ConstructOdometryPacket(byte* data);
};

#endif
