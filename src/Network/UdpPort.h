#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "thread.h"
#include <semaphore.h>
typedef unsigned char byte;

struct NetworkData{
    int size;
    byte* data;
};

class UdpPort : public Thread
{
    public:
        UdpPort();
		UdpPort(int MYPORT, bool isBroadcasting);
		~UdpPort();
        void sendData(NetworkData netData);
        NetworkData recieveData();
        int lastRecieved;
    private:
        bool isBroadcasting;
        void run();
        int sockfd;
	    struct sockaddr_in my_addr;    // my address information
	    struct sockaddr_in their_addr; // connector's address information
	    socklen_t addr_len;
	    int numBytes;

	    bool hasData;
        
        byte data[1024];

        sem_t mutex;

};

