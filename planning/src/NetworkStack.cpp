/*
* networkstack.cpp
* Author Morgan McClure
*/
#include <memory>
#include "NetworkStack.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>


float EndianSwap(char* input)
{
	float x;
	char buf[4];
	char *d = &buf[3];
	int i;
	for( i=0; i<4; i++)
	{
		*d-- = *input++;
	}
	memcpy(&x, buf, 4);
	return x;
}

float EndianSwap(float input)
{
	char buffer[4];
	memcpy(&buffer[0], &input, 4);
	return EndianSwap(&buffer[0]);
}

Network::Network()
{
    pthread_mutex_init(&(this->psoMutex),NULL);
	this->_state.x = 0.0;
	this->_state.x_variance = 0.0;
	this->_state.y = 0.0;
	this->_state.y_variance = 0.0;
	this->_state.theta = 0.0;
	this->_state.theta_variance = 0.0;
	this->exit = false;
//Unix sockets version:
	receiveSocketFd = 0;
	socketAddressSize = sizeof(struct sockaddr_in);
	bzero((char *) &receiverAddr, socketAddressSize);
	receiverAddr.sin_family = AF_INET;
	receiverAddr.sin_port = htons(LISTEN_PORT);
	receiverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	if((receiveSocketFd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
		perror("Error obtaining receiving socket");
	if((bind(receiveSocketFd, (struct sockaddr *) &receiverAddr, socketAddressSize)) < 0)
		perror("Error binding receiving socket");
	
	sendSocketFd = 0;
	bzero((char *) &sendAddr, socketAddressSize);
	sendAddr.sin_family = AF_INET;
	sendAddr.sin_port = htons(SEND_PORT);
	sendAddr.sin_addr.s_addr = (inet_addr("192.168.0.100"));
	
	int sockOpts=1;
	if((sendSocketFd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
		perror("Error obtaining socket file descriptor");
	if (setsockopt(sendSocketFd, SOL_SOCKET, SO_BROADCAST, (char *) &sockOpts, sizeof(sockOpts)) != 0)
		perror("Failed to obtain broadcast permissions");

//Set up the thread using pthreads
	pthread_attr_init(&pthread_custom_attr);
	printf("Calling create...\n");
	pthread_create(&listenThread, &pthread_custom_attr, EntryPoint, this);

}

/*Static*/
void * Network::EntryPoint(void * pt)
{
	Network * pthis = (Network*)pt;
	pthis->ListenUnix();
}


void Network::ListenUnix()
{
	struct sockaddr_in senderAddr;
	socklen_t senderAddrSize = sizeof(struct sockaddr_in);
	Command _pack;
	while(!this->exit) {
		ssize_t bytes = recvfrom(receiveSocketFd, (char *) &_pack, BUF_SIZE, 0, (struct sockaddr *) &senderAddr, &senderAddrSize);
		//printf("packet %d\n",(_pack.type));
		switch(_pack.type)
		{
			case POSE_t:
				Pose * point = (Pose *) &_pack;
				
				//endianswap
				point->x=EndianSwap(point->x);
				point->y = EndianSwap(point->y);
				point->theta=EndianSwap(point->theta);
				point->x_variance=EndianSwap(point->x_variance);
				point->y_variance=EndianSwap(point->y_variance);
				point->theta_variance=EndianSwap(point->theta_variance);
				point->front_sonar_ping=EndianSwap(point->front_sonar_ping);
				
				
				convertToMacPso(point);
				
				pthread_mutex_lock(&(this->psoMutex));
				
				_state=*point;
				pthread_mutex_unlock(&(this->psoMutex));
				
				break;
		//	default:
		//		printf("BAD PACKET");
		//		break;
		}
	}
}


void Network::convertToMacPso(Pose* pose){
  float temp=pose->x;
  pose->x=pose->y;
  pose->y=temp;
  pose->omega=-(pose->omega);
  pose->theta=2*3.1415-(pose->theta);
  //-(pose->theta+90.0*3.1415/180.0);
}
void Network::convertFromMacPSO(Pose* pose){
  float temp=pose->x;
  pose->x=pose->y;
  pose->y=temp;
  pose->omega=-(pose->omega);
  pose->theta=(2*3.1415-(pose->theta));//-90*3.1415/180; 
}

bool Network::sendWarning()
{
	printf("warning sent\n");

	AngularRateSpeedCommand _packet;
	_packet.type = ANGULARRATESPEEDCOMMAND_t;
	_packet.desiredAngularRate = EndianSwap(0.);
	_packet.desiredSpeed = EndianSwap(0.);
	sendto(sendSocketFd, (caddr_t) &_packet, sizeof(AngularRateSpeedCommand), 0,(struct sockaddr*) &sendAddr, socketAddressSize);

}
/*does not work!
void Network::sendWaypoint(float x, float y, float heading, float speed)
{
	WaypointCommand _packet;
	_packet.type = (WAYPOINTCOMMAND_t);
	//swap x and y to correct stuff
	_packet.y = EndianSwap(x);
	_packet.x = EndianSwap(y);
	//swap the heading info to me in crio frame
	_packet.heading = EndianSwap(-heading);
	_packet.max_speed = EndianSwap(speed);
	sendto(sendSocketFd, (caddr_t) &_packet, sizeof(WaypointCommand), 0,(struct sockaddr*) &sendAddr, socketAddressSize);
}
*/
/*
void Network::sendTweak(float deltaX, float deltaY, float deltaTheta)
{
	PSOTweakCommand _packet;
	_packet.type = htonl(PSOTWEAKCOMMAND_t);
	_packet.deltaX = EndianSwap(deltaX);
	_packet.deltaY = EndianSwap(deltaY);
	_packet.deltaTheta = EndianSwap(deltaTheta);
	sendto(sendSocketFd, (caddr_t) &_packet, sizeof(PSOTweakCommand), 0,(struct sockaddr*) &sendAddr, socketAddressSize);
}
*/

void Network::changeHeading(float head)
{
	HeadingSpeedCommand _packet;
	_packet.type = HEADINGSPEEDCOMMAND_t;
	_packet.desiredHeading = EndianSwap(head);
	_packet.desiredSpeed = EndianSwap(0.);
	sendto(sendSocketFd, (caddr_t) &_packet, sizeof(HeadingSpeedCommand), 0,(struct sockaddr*) &sendAddr, socketAddressSize);
}

void Network::angularRateSpeedCommand(float angularRate, float speed)
{
	printf("sending angular rate %f, %f\n",angularRate,speed);
	AngularRateSpeedCommand _packet;
	_packet.type = ANGULARRATESPEEDCOMMAND_t;
	
	_packet.desiredAngularRate = EndianSwap(-angularRate);
	_packet.desiredSpeed = EndianSwap(speed);
	
	sendto(sendSocketFd, (caddr_t) &_packet, sizeof(AngularRateSpeedCommand), 0,(struct sockaddr*) &sendAddr, socketAddressSize);
}

void Network::stop() {
	this->exit = true;
	pthread_join(listenThread, NULL);
	close(receiveSocketFd);
	close(sendSocketFd);
}

Pose Network::getCurrentPose()
{	
	pthread_mutex_lock(&(this->psoMutex));
	Pose temp=this->_state;
	pthread_mutex_unlock(&(this->psoMutex));
	
	return temp;
}

Network::~Network() {
	// TODO Auto-generated destructor stub
}


