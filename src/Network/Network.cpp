#include "Network.h"
#include "../Nao.h"
#include <iostream>
#include "../Kinematics/Kinematics.h"
#include "../CameraControl.h"
#include "GameController.h"
#include "../Behaviour/TeamBehaviour.h"
#include "../fieldCalculations.h"

GamePacket outgoingPacket;
GamePacket incomingPacket;

Network::Network(){
    std::cout << "---------Network Module Initialisation---------"<< endl;
    
    thelog << "NETWORK: Initialising Walk Port" << std::endl;
    udp_walk =  new UdpPort(WALK_PORT,false);
    thelog << "NETWORK: is listening for Walk on " << WALK_PORT << std::endl;
   
    std::cout << "--Start LocWM App Port--" << std::endl;
    udp_locWM =  new UdpPort(LOCWM_PORT,false);
    std::cout << "is waiting LocWM App Connection on " << LOCWM_PORT << std::endl;

    std::cout << "--Start Game Controller Port--" << std::endl;
    udp_gc = new UdpPort(GAMECONTROLLER_PORT,false);
    std::cout << "is listening for Game Controller on " << GAMECONTROLLER_PORT << std::endl;
    
    std::cout << "--Start GamePacket Port--" << std::endl;
    udp_GamePacket = new UdpPort(GAMEPACKET_PORT,true);
    std::cout << "is listening for Team Robot Info on "<< GAMEPACKET_PORT << std::endl;
    
    std::cout << "--Start Vision Port--" << std::endl;
    udp_Vision =  new UdpPort(VISION_PORT,false);
    std::cout << "Waiting on Vision App Connection on " <<  VISION_PORT << std::endl;

    std::cout << "--Start Classified Port--" << std::endl;
    udp_Classified = new UdpPort(CLASSIFIED_PORT,false);
    std::cout << "is waiting Classified Viewer App Connection on "<< CLASSIFIED_PORT << std::endl;


    std::cout << "---------Network Module Initialisatised---------"<< std::endl;

    //SET ALL ROBOTPACKETS TO "NO NEW INFO"

    for (int i = 0; i < NUM_ROBOTS; i++)
    {
        latestReceivedGamePackets[i].processedWM = true;
        latestReceivedGamePackets[i].processedBVR = true;
        latestReceivedGamePackets[i].packet.packetID = -1;
    }

    GControlData = NULL;
    locWMStream=false;
    visionStream=false;
    packetsSent = 0;
    imageClass = new NUbotImage();
	return;
}

Network::~Network(){

    delete udp_gc;
    delete udp_GamePacket;
    delete udp_locWM;
    delete udp_Classified;
   
    //delete udp_Vision;
    cout << "UDPPorts Destroyed" << endl;
}

void Network::ProcessUpdates(){
    CheckGameController();
    //CheckGamePacket();
    //CheckWalkPacket();
}

void Network::CheckGameController()
{
    //std::cout << "GC Packet Check.." << endl;
    NetworkData gc_rawData = udp_gc->recieveData();
    framesSinceGameController ++;

    GameController* gc = &GameController::getInstance();
    bool newPacket = gc->checkForManualInput();

    if (newPacket) {
        RoboCupGameControlReturnData returnPacket = gc->getReturnPacket();
        // TODO: Need to send the packet.
    }

    if (gc_rawData.size == -2)
        std::cout << "Error: GCPort: Select" << endl;
    if (gc_rawData.size == -3)
        std::cout << "Error: GCPort: Recieve" << endl;
    if (gc_rawData.size > -1){
        framesSinceGameController = 0;
        GControlData = (RoboCupGameControlData*) gc_rawData.data;
        gc->processPacket(GControlData);
    }
    
	return;
}

void Network::CheckGamePacket()
{
    GameController* gc = &GameController::getInstance();

    NetworkData gamePack = udp_GamePacket->recieveData();
    if (gamePack.size == sizeof(GamePacket))
    {
        memcpy(&incomingPacket,gamePack.data,sizeof(GamePacket));
        int inRobotID = (int)incomingPacket.robotNumber;
//        cout << " Got Packet from Another Robot: "<< inRobotID <<endl;
        if ((int)incomingPacket.team != gc->getTeamColour())  {
            cout << "Got a incorrect team packet - Somebody is on the wrong team !" << endl << flush;
            return;
        }

        if(inRobotID == gc->getPlayerNumber())
        {
            //cout << "Ignoring Self-Packet from " << inRobotID <<endl << flush;
            memcpy(&latestReceivedGamePackets[inRobotID-1].packet,&incomingPacket,sizeof(GamePacket));
            latestReceivedGamePackets[inRobotID-1].processedWM=true;                               // Set to true so WM will NOT incorporate the new data
            latestReceivedGamePackets[inRobotID-1].processedBVR=true;
            return;
        }
        // Store the packet for each robots in its position in the latest packet array
        //  ***** THIS IS WHERE WE CAN CHECK FOR OUT-OF-SYNC Packets *****
   
      
        if (incomingPacket.packetID < latestReceivedGamePackets[inRobotID-1].packet.packetID) {
            cout << "Ignoring out-of-order packet from " << inRobotID << " with "<< incomingPacket.packetID <<endl << flush;
            memcpy(&latestReceivedGamePackets[inRobotID-1].packet,&incomingPacket,sizeof(GamePacket));
            latestReceivedGamePackets[inRobotID-1].processedWM=true;                               // Set to true so WM will NOT incorporate the new data
            latestReceivedGamePackets[inRobotID-1].processedBVR=true;
            return;
        }  

      if (incomingPacket.packetID == latestReceivedGamePackets[inRobotID-1].packet.packetID) {
        cout << "Ignoring repeated packet from " << inRobotID << endl << flush;
        memcpy(&latestReceivedGamePackets[inRobotID-1].packet,&incomingPacket,sizeof(GamePacket));
        latestReceivedGamePackets[inRobotID-1].processedWM=true;
        latestReceivedGamePackets[inRobotID-1].processedBVR=true;
        return; 
      }
      if (incomingPacket.frameNumber > 0) { // Its packet numbered by game time .. so we can try and throw out old packets
        if ( ( incomingPacket.frameNumber - gc->getRemainingGameTime() ) > 5) { // Ignore any message received from a robot that was sent more than 5 seconds ago
          cout << "Ignoring old packet (G) from " << inRobotID << " (" << incomingPacket.frameNumber << " - " << gc->getRemainingGameTime() << ")"<<  endl << flush;
          memcpy(&latestReceivedGamePackets[inRobotID-1].packet,&incomingPacket,sizeof(GamePacket));
          latestReceivedGamePackets[inRobotID-1].processedWM=true;
          latestReceivedGamePackets[inRobotID-1].processedBVR=true;
          return; 
        }
      } else {
        incomingPacket.frameNumber*=-1;
      }
      
      memcpy(&latestReceivedGamePackets[inRobotID-1].packet,&incomingPacket,sizeof(GamePacket));
      latestReceivedGamePackets[inRobotID-1].processedWM=false;                                   // Set to false so WM will incorporate the new data
      latestReceivedGamePackets[inRobotID-1].processedBVR=false;                                  // Set to false so BVR will incorporate the new data

      // Florians Passing
      //int index = (int)incomingPacket.robotNumber-1;
      //int passCmd = incomingPacket.passMessage;
      //int* incommingPassMessages = new int[ROBOTNUMBER]
      //incomingPassMessages[index]=passCmd;

      //int s = passCmd / 100;
      //int r = (passCmd-s*100)/10;
      //int m = passCmd-s*100-r*10; // my pass msg

      //if (r == ROBOTNUMBER-1 && m == 3) { // Receiver = me and Sender just kicked a pass
      //  locWM_.passKickedBy = s;
      //  cout << "Sender just kicked the ball towards me (I think)" << endl << flush;
      //}
    }
}

void Network::CheckWalkPacket()
{
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
    }
}

void Network::SendUpdate(){

    SendLocWMPacket();
    SendVisionPacket();
    SendClassifiedPacket();
    SendGamePacket();
	return;
}

void Network::SendGamePacket()
{
    GameController* gc = &GameController::getInstance();
    if( (NAO->FrameCounter)%5 == 0)//Send Once Every 5 Frames.. AW
    {
        if(gc->isPenalised() == false) {
//            std::cout << "Sending GamePacket at " << NAO->FrameCounter <<std::endl;
            NetworkData gamePacket;
            int sizePacket = sizeof(GamePacket);
            gamePacket.data = new byte[sizePacket];
            //cout << "Creating Wireless FieldObjects" << endl;
            sizePacket = ConstructTeamPacket(gamePacket.data);
            gamePacket.size = sizePacket;
            //cout << "Sending Wireless FieldObjects" << endl;
            udp_GamePacket->sendData(gamePacket);
            //cout << "Wireless FieldObjects sent" << endl;
            delete [] gamePacket.data;
        }
    }
}

void Network::SendClassifiedPacket()
{
    NetworkData visionData;
    visionData.data = NULL;
    int packetSize;

    NetworkData classified_rawData = udp_Classified->recieveData();

    if(classified_rawData.size > -1)
    {
        char* data;
        data = (char*)classified_rawData.data;
        if(*data == STARTSTREAM)
        {
            classStream = true;
            std::cout <<"Start Stream Classified Data.. ("<< *data << ")"<< endl;
        }
        else if (*data == SENDPACKET )
        {
            classStream = false;
            std::cout <<"Sending Single Classified Data..("<< *data << ")"<< endl;
            int sizePacket = sizeof(int)*3 + IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(uint8);
            visionData.data = new byte[sizePacket];
            packetSize = ConstructNewClassifiedImagePacket(visionData.data);
            visionData.size = packetSize;
            udp_Classified->sendData(visionData);
            std::cout << "Sent Classified Packet.." << std::endl;
        }
        else
        {
            classStream = false;
            std::cout <<"Stop Stream Classified Data..("<< *data << ")"<< endl;
        }

    }    
    if (classStream == true)
    {
            int sizePacket = sizeof(int)*3 + IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(uint8);
            visionData.data = new byte[sizePacket];
            packetSize = ConstructNewClassifiedImagePacket(visionData.data);
            visionData.size = packetSize;
            udp_Classified->sendData(visionData);
    }
    delete[] visionData.data;
    return;
}


void Network::SendVisionPacket()
{
    NetworkData visionData;
    visionData.data = NULL;
    int packetSize;
    byte* toSenddata = NULL;
    //std::cout << "Vision Packet Check.." << std::endl;
    NetworkData vision_rawData = udp_Vision->recieveData();
    //std::cout << "Vision Packet Check Finnished.." << std::endl;
    if(vision_rawData.size > -1)
    {  
        char* data;
        data = (char*)vision_rawData.data;
        if(*data == STARTSTREAM)
        {
            visionStream = true;
            std::cout <<"Start Stream Vision Data.. ("<< *data << ")"<< endl;
        }
        else if (*data == SENDPACKET)
        {
            std::cout << "Creating VisionPacket.." << std::endl;
            //int sizePacket = sizeof(int)*3 + IMAGE_WIDTH*IMAGE_HEIGHT*2*sizeof(uint8) + B_NUM_SENSORS*sizeof(float) +J_NUM_JOINTS*sizeof(float);
            int sizePacket = imageClass->calculateFrameLength(T_YUV, IMAGE_WIDTH, IMAGE_HEIGHT, J_NUM_JOINTS, B_NUM_SENSORS, T_NUM_SENSORS);
            toSenddata = new byte[sizePacket];
            packetSize = ConstructNewVisionPacket(toSenddata);
            std::cout << "Finnsihed Creating VisionPacket.. " << packetSize << std::endl;
            if (IMAGE_HEIGHT == 240 && IMAGE_WIDTH == 320)
            {
                int segmentSize = 30000;
                int segment = (int)ceil(packetSize/(float)segmentSize);
                for (int i = 0; i< segment; i++)
                {
                    if (i == segment-1)
                    {
                        visionData.size = packetSize - segmentSize*i;
                    }
                    else
                    {
                        visionData.size = segmentSize;
                    }
                    
                    visionData.data = &toSenddata[segmentSize*i];
                    udp_Vision->sendData(visionData);
                }
            }
            else
            {
                visionData.size = packetSize;
                visionData.data = toSenddata;
                udp_Vision->sendData(visionData);
            }
            std::cout << "Sent VisionPacket.." << std::endl;
        }
        else
        {
            visionStream = false;
            std::cout <<"Stop Stream Vision Data..("<< *data << ")"<< endl;
        }
    }
    if (visionStream == true && NAO->FrameCounter%10 ==0)
    {
        std::cout << "Creating VisionPacket.." << std::endl;
        //int sizePacket = sizeof(int)*3 + IMAGE_WIDTH*IMAGE_HEIGHT*2*sizeof(uint8) + B_NUM_SENSORS*sizeof(float) +J_NUM_JOINTS*sizeof(float);
        int sizePacket = imageClass->calculateFrameLength(T_YUV, IMAGE_WIDTH, IMAGE_HEIGHT, J_NUM_JOINTS, B_NUM_SENSORS, T_NUM_SENSORS);
        std::cout << "Size Of Packet: " << sizePacket << endl;
        toSenddata = new byte[sizePacket];
        packetSize = ConstructNewVisionPacket(toSenddata);
        std::cout << "Finnsihed Creating VisionPacket.. " << packetSize << std::endl;
        if (IMAGE_HEIGHT == 240 && IMAGE_WIDTH == 320)
        {
            int segmentSize = 30000;
            int segment = (int)ceil(packetSize/(float)segmentSize);
            for (int i = 0; i< segment; i++)
            {
                if (i == segment-1)
                {
                    visionData.size = packetSize - segmentSize*i;
                }
                else
                {
                    visionData.size = segmentSize;
                }
                
                visionData.data = &toSenddata[segmentSize*i];
                udp_Vision->sendData(visionData);
            }
        }
        else
        {
            visionData.size = packetSize;
            visionData.data = toSenddata;
            udp_Vision->sendData(visionData);
        }
        std::cout << "Sent VisionPacket.." << std::endl;
    }
    //std::cout << "Vision Packet Finnished.." << std::endl;
    delete[] toSenddata;
    return;
}
void Network::SendLocWMPacket()
{
    NetworkData locWMdata;
    locWMdata.data = NULL;
    int packetSize;

    //std::cout << "LocWM Packet Check.." ;
    NetworkData locWM_rawData = udp_locWM->recieveData();
    if (locWM_rawData.size > -1)
    {
        char* data;
        data = (char*)locWM_rawData.data;
        
        if(*data == STARTSTREAM)
        {
            locWMStream = true;
            std::cout <<"Start Stream LocWM Data.. ("<< *data << ")"<< endl;
        }
        else if (*data == SENDPACKET )
        {
            locWMStream = false;
            
            locWMdata.data = new byte[1024*2];
            locWMdata.size = -1;
            locWMdata.size = ConstructNewWMDataPacket(locWMdata.data);
            udp_locWM->sendData(locWMdata);
            std::cout <<"Sending Single LocWM Data..("<< *data << ")"<< locWMdata.size << endl;
        }
        else
        {
            locWMStream = false;
            std::cout <<"Stop Stream LocWM Data..("<< *data << ")"<< endl;
        }

    }    
    if (locWMStream == true)
    {
        locWMdata.data = new byte[1024*2];
        locWMdata.size = -1;
        locWMdata.size = ConstructNewWMDataPacket(locWMdata.data);
        udp_locWM->sendData(locWMdata);
    }
    delete[] locWMdata.data;
    return;
}

int Network::ConstructNewVisionPacket(byte* data){
/*
    int packetOffset = 0;
    double joints[J_NUM_JOINTS], sensors[SE_NUM_SENSORS];
    NAO->ReadSensorsToBuffer(sensors, joints);
    //cout << joints[J_NUM_JOINTS] << "," << joints[J_NUM_JOINTS] <<endl;
    memcpy(data+packetOffset, &(NAO->FrameCounter) , sizeof(int));
    packetOffset += sizeof(int);

    cout << NAO->FrameCounter << ": "<< NAO->m_vision->numBlobs << " Blobs formed."<< endl;

    memcpy(data+packetOffset, &(IMAGE_WIDTH) , sizeof(int));
    packetOffset += sizeof(int);

    memcpy(data+packetOffset, &(IMAGE_HEIGHT) , sizeof(int));
    packetOffset += sizeof(int);
    
    memcpy(data+packetOffset, NAO->m_vision->rawImage , IMAGE_WIDTH*IMAGE_HEIGHT*2*sizeof(uint8));
    packetOffset += IMAGE_WIDTH*IMAGE_HEIGHT*2*sizeof(uint8);

    memcpy(data+packetOffset, balanceValues , B_NUM_SENSORS*sizeof(float));
    packetOffset += B_NUM_SENSORS*sizeof(float);
    
    memcpy(data+packetOffset, jointPositions , J_NUM_JOINTS*sizeof(float));
    packetOffset += J_NUM_JOINTS*sizeof(float);   
*/
    
    NaoCamera CurrentCamera = CameraControl::getInstance().getCameraInUse();
    std::cout << "Got Camera " << CurrentCamera<<std::endl;
    char* buffer = (char*)data;
    std::cout << "Making Buffer "<<std::endl;
    int packetOffset = imageClass->writeToBuffer(buffer, NAO->FrameCounter, CurrentCamera, (uint8*)NAO->m_vision->rawImage, jointPositions, balanceValues, touchValues);
            
    return packetOffset;    
}

int Network::ConstructNewWMDataPacket(byte* data) 
{
    int packetOffset = 0;
    int numActiveModels=0;
  
    GameController* gc = &GameController::getInstance();

    //we'll put the size of the packet in this spot
    memcpy(data+packetOffset, &packetOffset , sizeof(int));
    packetOffset += sizeof(int);

    //the frame number.
    memcpy(data+packetOffset, &(NAO->FrameCounter), sizeof(int));
    packetOffset += sizeof(int);

    //your number and team (RED is negative ROBOTNUMBER, BLUE is positive ROBOTNUMBER)
    int sendRobot = gc->getPlayerNumber();
    if (gc->getTeamColour() == TEAM_RED) sendRobot*=-1;
    memcpy(data+packetOffset, &sendRobot, sizeof(int));
    packetOffset += sizeof(int);
    KF* models = NAO->m_locWM.models;
    //number of models and the entires
    for (int i = 0; i < LocWM::c_MAX_MODELS; i++) {
        if (models[i].isActive) {
            numActiveModels++;
        }
    }

    memcpy(data+packetOffset, &numActiveModels, sizeof(int));
    packetOffset += sizeof(int);
    //memcpy(data+packetOffset, &locWM_.bestModel, sizeof(int));
    int bestmodel = NAO->m_locWM.getBestModelID();
    memcpy(data+packetOffset, &bestmodel, sizeof(int));
    packetOffset += sizeof(int);

 
    double sdX, sdY, sdOr, sdbX, sdbY;
    double posX, posY, posHeading, ballX, ballY;
    for (int i = 0; i < LocWM::c_MAX_MODELS; i++) {
        if (models[i].isActive) {
            sdX = models[i].sd(0);
            sdY = models[i].sd(1);
            sdOr = models[i].sd(2);
            sdbX = models[i].sd(3);
            sdbY = models[i].sd(4);
        
            posX = models[i].getState(0);
            posY = models[i].getState(1);
            posHeading = models[i].getState(2);
            ballX = models[i].getState(3);
            ballY = models[i].getState(4);
        
            memcpy(data+packetOffset, &i, sizeof(int));
            packetOffset+=sizeof(int);
            memcpy(data+packetOffset, &models[i].alpha, sizeof(double));
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &posX, sizeof(double));   // X
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &posY, sizeof(double));   // Y
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &posHeading, sizeof(double));   // Heading
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &sdX, sizeof(double));   // varX
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &sdY, sizeof(double));   // varY
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &sdOr, sizeof(double));   // varHeading
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &ballX, sizeof(double));   // ball X
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &ballY, sizeof(double));   // ball Y
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &sdbX, sizeof(double));   // var ball X
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &sdbY, sizeof(double));   // var ball Y
            packetOffset+=sizeof(double);
        }
    }

    // Send stuff you have seen
    int seenCount = 0;
    for(int k = FO_BALL; k < FO_PENALTY_BLUE; k++){
        if (fieldObjects[k].seen == 1) seenCount++;
    }
    memcpy(data+packetOffset, &seenCount, sizeof(int));
    packetOffset += sizeof(int);

    for(int k = FO_BALL; k < FO_PENALTY_BLUE; k++){
        if(fieldObjects[k].seen == 1){
            memcpy(data+packetOffset, &k, sizeof(int));   // Object ID
            packetOffset+=sizeof(int);
            memcpy(data+packetOffset, &fieldObjects[k].visionDistance, sizeof(double));
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &fieldObjects[k].visionElevation, sizeof(double));
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &fieldObjects[k].visionBearing, sizeof(double));
            packetOffset+=sizeof(double);
            memcpy(data+packetOffset, &fieldObjects[k].locationConfidence, sizeof(double));
            packetOffset+=sizeof(double);

            if (k>=  FO_BLUE_GOAL && k<= FO_YELLOW_GOALPOST_UNKNOWN) { // send second distance for goals
                memcpy(data+packetOffset, &fieldObjects[k].D2PDistance, sizeof(double));
                packetOffset+=sizeof(double);
                memcpy(data+packetOffset, &fieldObjects[k].D2PElevation, sizeof(double));
                packetOffset+=sizeof(double);
                memcpy(data+packetOffset, &fieldObjects[k].D2PBearing, sizeof(double));
                packetOffset+=sizeof(double);
            }
        }
    }
    // send odometry
    packetOffset+=ConstructOdometryPacketExtended(data+packetOffset);
    // send extra
    packetOffset+=ConstructWMExtraPacket(data+packetOffset);
    memcpy(data, &packetOffset, sizeof(int));
  
    return packetOffset;
}


int Network::ConstructOdometryPacket(byte* data)
{
    int packetOffset = 0;
    memcpy(data, &odomForward, sizeof(double));
    packetOffset+=sizeof(double);
    memcpy(data+packetOffset, &odomLeft, sizeof(double));
    packetOffset+=sizeof(double);
    memcpy(data+packetOffset, &odomTurn, sizeof(double));
    packetOffset+=sizeof(double);

    return packetOffset;
}

// extended to have the reliability as well - for World Model app. SY
int Network::ConstructOdometryPacketExtended(byte* data)
{
    int packetOffset = 0;
    memcpy(data, &odomForward, sizeof(double));
    packetOffset+=sizeof(double);
    memcpy(data+packetOffset, &odomLeft, sizeof(double));
    packetOffset+=sizeof(double);
    memcpy(data+packetOffset, &odomTurn, sizeof(double));
    packetOffset+=sizeof(double);
    memcpy(data+packetOffset, &odomReliability, sizeof(double));
    packetOffset+=sizeof(double);
    return packetOffset;
}

int Network::ConstructWMExtraPacket(byte* data)
{
    KF* locWM_ = NAO->m_locWM.models;
    // Send EXTRA localisation entries
    int packetOffset = 0;
    //int t1 = (int)(locWM_->ballGrabbed); //AIBO CODE
    int t1 =0;
    memcpy(data+packetOffset, &t1, sizeof(int));
    packetOffset+=sizeof(int);

    //int t2 = (int)nearEndOfKick; //AIBO CODE
    int t2 =0;
    memcpy(data+packetOffset, &t2, sizeof(int));
    packetOffset+=sizeof(int);

    // send Head angles
    Kinematics* kin = &Kinematics::getInstance();
    //double tilt = MICRO_TO_RAD(sensorValues_[S_HEAD_TILT_SMALL]+sensorValues_[S_HEAD_TILT_BIG]);
    double tilt = (double)kin->GetHeadPitch();
    memcpy(data+packetOffset, &tilt, sizeof(double));
    packetOffset+=sizeof(double);
    //double pan = MICRO_TO_RAD(sensorValues_[S_HEAD_PAN]);
    double pan = (double)kin->GetHeadYaw();
    memcpy(data+packetOffset, &pan, sizeof(double));
    packetOffset+=sizeof(double);
    return packetOffset;
}

int Network::ConstructNewClassifiedImagePacket(byte* data) {
    int packetOffset = 0;
    memcpy(data+packetOffset, &(NAO->FrameCounter) , sizeof(int));
    packetOffset += sizeof(int);
    memcpy(data+packetOffset, &(IMAGE_WIDTH) , sizeof(int));
    packetOffset += sizeof(int);
    memcpy(data+packetOffset, &(IMAGE_HEIGHT) , sizeof(int));
    packetOffset += sizeof(int);
    memcpy(data+packetOffset, NAO->m_vision->classifiedImage , IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(uint8));
    packetOffset += IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(uint8);
    
    return packetOffset;
}


int Network::ConstructTeamPacket(byte* data) 
{
    // Fill header data
    GameController* gc = &GameController::getInstance();

    outgoingPacket.headerID='G';
    outgoingPacket.robotNumber = (byte)gc->getPlayerNumber();
    outgoingPacket.team = (byte)gc->getTeamColour();
    outgoingPacket.frameNumber = -1*(NAO->FrameCounter);
    outgoingPacket.packetID = packetsSent;
    //if (framesSinceGameController < FRAMESINCEGC) {  //heard from game controller in the last 15 seconds (15*30fps)  This framesSince shit is new as of 18/5/07 (MQ)
    //  outgoingPacket.frameNumber = (int)(gameTime-framesSinceGameController/30.0);  // the game time + the estimated time passed since I heard from game controller
  //}

  // Loc Info
    fieldObjects[FO_TEAM1].ConstructWireless();
    fieldObjects[FO_BALL].ConstructWireless();
    KF* locWM_ = NAO->m_locWM.models;
    int bestmodel = NAO->m_locWM.getBestModelID();
    //FOR NEW SHARED BALL INFORMATION!
    //fieldObjects[FO_BALL].wireless.SRXX = locWM_[bestmodel].stateStandardDeviations[3][3];
    //fieldObjects[FO_BALL].wireless.SRXY = locWM_[bestmodel].stateStandardDeviations[3][4];
    //fieldObjects[FO_BALL].wireless.SRYY = locWM_[bestmodel].stateStandardDeviations[4][4];

    Matrix tempBallSR = locWM_[bestmodel].GetBallSR();
    fieldObjects[FO_BALL].wireless.SRXX = tempBallSR[0][0];
    fieldObjects[FO_BALL].wireless.SRXY = tempBallSR[0][1];
    fieldObjects[FO_BALL].wireless.SRYY = tempBallSR[1][1];
    outgoingPacket.self=fieldObjects[FO_TEAM1].wireless;
    outgoingPacket.ball=fieldObjects[FO_BALL].wireless;

    // BVR Info
    double bvrMessage;
    if(gc->getPlayerNumber() == 1){
        if(TeamBehaviour::getInstance().iAmChasing == true){
            bvrMessage = 20.0f;
        }
        else {
            bvrMessage = 1000.0f;
        }
    }
    else {
        if (fieldObjects[FO_BALL].seen == true){
            bvrMessage = fieldObjects[FO_BALL].visionDistance * cos(fieldObjects[FO_BALL].visionElevation);
        }
        else if (isTheBallLost() == false){
            // If the robot cannot see the ball, but thinks it knows where it is, send the estimated distance.
            bvrMessage = fieldObjects[FO_BALL].wmDistance;
        } else {
            bvrMessage = 1000.0f;
        }
    }

    outgoingPacket.bvrMessage1=bvrMessage;

    // Pass Info
    int passMessage = 0;
    outgoingPacket.passMessage=passMessage;
    memcpy(data,&outgoingPacket,sizeof(GamePacket));
    packetsSent++;
    return sizeof(GamePacket);
}
