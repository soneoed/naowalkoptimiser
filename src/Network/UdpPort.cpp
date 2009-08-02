#include "UdpPort.h"
#include <sstream>
#include <iostream>

#define TEAM_BROADCAST_ADDRESS "192.168.255.255"
#define TEAM_PLAYER1_ADDRESS "192.168.16.11"
#define TEAM_PLAYER2_ADDRESS "192.168.16.12"
#define TEAM_PLAYER3_ADDRESS "192.168.16.13"
#define TEAM_PLAYER4_ADDRESS "192.168.16.14"

UdpPort::UdpPort(): Thread("Blank")
{
    
}


UdpPort::UdpPort(int MYPORT,bool isBroadcast): Thread("UDP Thread"){
    
    printf("Setting up socket..\n");
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) 
    {
        perror("socket");
		exit(1);
	}

    isBroadcasting = isBroadcast;

    if(isBroadcasting)
    {
            int broadcast = 1;
            std::cout << "Is Setting to BroadCast on " << MYPORT << std::endl;
            if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast,
                sizeof broadcast) == -1) {
                    perror("setsockopt (SO_BROADCAST)");
                    exit(1);
            }
            their_addr.sin_family = AF_INET;     // host byte order
            their_addr.sin_port = htons(MYPORT); // short, network byte order
            their_addr.sin_addr.s_addr = inet_addr(TEAM_BROADCAST_ADDRESS);
            memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);
    }

        
    my_addr.sin_family = AF_INET;         // host byte order
    my_addr.sin_port = htons(MYPORT);     // short, network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
    memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);
    
    printf("Binding socket..\n");
	if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof my_addr) == -1) {
		perror("bind");
		exit(1);
	}
    
    lastRecieved = 0;
    hasData = false;
    sem_init(&mutex, 0, 1);
    if(isBroadcasting)
    {
        sendto(sockfd,'\0',1,0,(struct sockaddr *)&their_addr,sizeof(their_addr));
    }
    
    start();
}

UdpPort::~UdpPort(){
    close(sockfd);
}
void UdpPort::run()
{
    struct sockaddr_in local_their_addr; // connector's address information
    socklen_t local_addr_len;
    byte localdata[1024];
    int localnumBytes;
    while(1)
    {

        localnumBytes = recvfrom(sockfd, localdata, 1024 , 0,(struct sockaddr *)&local_their_addr, &local_addr_len);
        //printf("Packet Recieved..\n");
        sem_wait (&mutex);
        their_addr = local_their_addr;
        addr_len = local_addr_len;
        memcpy(data,localdata,1024);
        numBytes = localnumBytes; 
        hasData = true;
        sem_post (&mutex);

    }
    return;
}

NetworkData UdpPort::recieveData(){

    NetworkData netData;
    netData.size = -1;
    netData.data = NULL; 
    sem_wait (&mutex);
    if (hasData == true)
    {
        hasData = false;
        netData.size = numBytes;
        netData.data = data;  
    }
    sem_post (&mutex);
    
    return netData;
}

void UdpPort::sendData(NetworkData netData)
{
    sem_wait (&mutex);
    if (isBroadcasting)
    {
        their_addr.sin_addr.s_addr = inet_addr(TEAM_PLAYER1_ADDRESS);
        sendto(sockfd,netData.data,netData.size,0,(struct sockaddr *)&their_addr,sizeof(their_addr));
        their_addr.sin_addr.s_addr = inet_addr(TEAM_PLAYER2_ADDRESS);
        sendto(sockfd,netData.data,netData.size,0,(struct sockaddr *)&their_addr,sizeof(their_addr));
        their_addr.sin_addr.s_addr = inet_addr(TEAM_PLAYER3_ADDRESS);
        sendto(sockfd,netData.data,netData.size,0,(struct sockaddr *)&their_addr,sizeof(their_addr));
        their_addr.sin_addr.s_addr = inet_addr(TEAM_PLAYER4_ADDRESS);
        sendto(sockfd,netData.data,netData.size,0,(struct sockaddr *)&their_addr,sizeof(their_addr));
        
    }
    else
    {
        sendto(sockfd,netData.data,netData.size,0,(struct sockaddr *)&their_addr,sizeof(their_addr));
    }
    

    sem_post (&mutex);

    //printf("sent %d bytes to %s\n", netData.size, inet_ntoa(their_addr.sin_addr));

    return;

}
