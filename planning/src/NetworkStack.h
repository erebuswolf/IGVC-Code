/*
 * networkstack.h
 * Author Morgan McClure
 */


#ifndef LISTENER_H
#define LISTENER_H

#define LISTEN_PORT 50000
#define SEND_PORT 50001

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "packets.h"
/*
typedef struct {
	uint32_t type;
	char commandData[BUF_SIZE - sizeof(int8_t)];
} Packet;*/

class Network {
public:
	Network();
	Pose getCurrentPose();
	void stop();
	virtual ~Network();
	bool sendWarning();
	void changeHeading(float head);
	//void sendWaypoint(float x, float y, float heading, float speed);
	//void sendTweak(float deltaX, float deltaY, float deltaTheta);
	
	void angularRateSpeedCommand(float , float );
protected:
	static void * EntryPoint(void*);
	void ListenUnix();
private:
	socklen_t socketAddressSize;
	pthread_mutex_t psoMutex;

	struct sockaddr_in receiverAddr;
	struct sockaddr_in sendAddr;

	int receiveSocketFd;
	int sendSocketFd;

	pthread_t listenThread;
	pthread_attr_t pthread_custom_attr;
	
	Pose _state;
	bool exit;
	void convertToMacPso(Pose* pose);
	void convertFromMacPSO(Pose* pose);
};

#endif /* LISTENER_H */
