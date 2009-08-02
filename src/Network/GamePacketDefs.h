#ifndef GAME_PACKET_DEFS_DEFINED
#define GAME_PACKET_DEFS_DEFINED

typedef unsigned char uint8;

struct WirelessFieldObj {
  int seen;
  double x;
  double y;
  double orientation;
  double sdx;
  double sdy;
  double sdtheta;
  double SRXX; //For New LOCWM SharedBall
  double SRXY; //For New LOCWM SharedBall
  double SRYY; //For New LOCWM SharedBall
};

struct GamePacket {
  // Packet header
  char headerID;
  int frameNumber;
  int packetID;
  uint8 robotNumber;
  uint8 team;
  // Actual Data
  double bvrMessage1; // In this case the `hacked' ball distance
  int passMessage; // Florians Message
  WirelessFieldObj self;
  WirelessFieldObj ball;
};

struct StoredGamePackets {
  bool processedWM;
  bool processedBVR;
  GamePacket packet;
};

#endif
