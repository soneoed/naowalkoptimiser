#include <iostream>
#include <sstream>
#include "Network.h"

Network::Network()
{
    thelog << "---------Network Module Initialisation---------"<< endl;
    
    thelog << "NETWORK: Initialising Walk Port" << std::endl;
    udp_walk =  new UdpPort(WALK_PORT,false);
    thelog << "NETWORK: is listening for Walk on " << WALK_PORT << std::endl;

    thelog << "---------Network Module Initialisatised---------"<< std::endl;


    packetsSent = 0;
	return;
}

Network::~Network(){

    delete udp_walk;
   
    //delete udp_Vision;
    thelog << "UDPPorts Destroyed" << endl;
}

void Network::ProcessUpdates(){
    CheckWalkPacket();
}

void Network::CheckWalkPacket()
{
    static short callssincereceived = 0;
    NetworkData walk_rawData = udp_walk->recieveData();
    if (walk_rawData.size > -1)
    {
        string packet = string((char*)walk_rawData.data, walk_rawData.size);
        float values[10];
        unsigned char numvalues = 0;
        short pos = 0;       // the current index into s
        
        while (pos > -1)
        {
            values[numvalues] = atof(packet.substr(pos).c_str());
            numvalues++;
            pos = packet.find(",", pos);
            
            if (pos == string::npos)
                pos = -1;
            else
                pos++;
        }
        thelog << "NETWORK: CheckWalkPacket" << endl;
        for (unsigned char i=0; i<numvalues; i++)
            thelog << values[i] << ", ";
        thelog << endl;
        
        if (numvalues == 6)
        {
            callssincereceived = 0;
            networkControl1 = values[3];
            networkControl2 = values[4];
            networkControl3 = values[5];
        }
        else
            callssincereceived++;
    }
    else
        callssincereceived++;
    
    // Deadman handle; if the connection is lost stop the robot
    if (callssincereceived > 5)
    {
        thelog << "NETWORK: Not connected to laptop!" << endl;
        networkControl1 = -1000;
        networkControl2 = -1000;
        networkControl3 = -1000;
    }

}

void Network::SendUpdate()
{
    SendOdometryPacket();
	return;
}

void Network::SendOdometryPacket()
{
    NetworkData ododata;
    ododata.data = NULL;
    int packetSize;
    
    ododata.data = new byte[1024*2];
    ododata.size = -1;
    ododata.size = ConstructOdometryPacket(ododata.data);
    udp_walk->sendData(ododata);

    delete[] ododata.data;
    return;
}

int Network::ConstructOdometryPacket(byte* data)
{
    int packetOffset = 0;
    stringstream ss;
    ss << odometryDeltaX << ", " << odometryDeltaY << ", " << odometryDeltaO << endl;
    byte* temp = (byte*)ss.str().c_str();
    short length = ss.str().length();
    memcpy(data, temp, sizeof(byte)*length);
    packetOffset+=sizeof(byte)*length;
    
    return packetOffset;
}
