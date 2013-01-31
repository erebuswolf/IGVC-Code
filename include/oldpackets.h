/* 
 * File:   packets.h
 * Author: Eric Perko/Morgan McClure
 * Created on August 31, 2009, 12:36 PM
 */

#ifndef _PACKETS_H
#define	_PACKETS_H

#ifdef	__cplusplus
extern "C" {
#endif

    #define BUF_SIZE 1024

    /*
     * These types are defined to numbers that match the type code in the packets
     * themselves. This is why they are defined to numbers
     */

	#define REBOOTCOMMAND_t 0
	#define HEADINGSPEEDCOMMAND_t 1
	#define WAYPOINTCOMMAND_t 2
	#define STARTCOMMAND_t 3
	#define STOPCOMMAND_t 4
	#define QUERYSYSTEMSTATUS_t 5
	#define ESTOP_t 6
	#define DIAGNOSTICS_t 7
	#define STEERINGSWITCH_t 10
	#define VECTORCOMMAND_t 11
	#define ANGULARFORWARDCOMMAND_t 12
	#define PSOTWEAKCOMMAND_t 15
	#define POSE_t 32
	#define ACK_t 33
	#define SYSINFO_t 34	

/*The all encompassing packet type*/
	typedef struct Command_t {
		uint32_t type;
		char commandData[BUF_SIZE - sizeof(int8_t)];
	} Command;

/*Used to change the steering mode on the fly*/
	typedef struct SteeringModeCommand_t {
		uint32_t type;
		uint32_t newMode;
	} SteeringModeCommand;

/*Packet to be broadcast from the PSO*/
	typedef struct Pose_t{
		uint32_t type;
		float x;
		float y;
		float theta;
		float xvar;
		float yvar;
		float thetavar;
	} PosePacket;

/*This packet is defined to give a desired heading and forward velocity*/
	typedef struct HeadingSpeedCommand_t {
		uint32_t type;
		float desiredHeading;
		float desiredSpeed;
	} HeadingSpeedCommand;

/*This packet is for commanding raw angular and forward velocity*/
	typedef struct AngularForwardCommand_t {
		uint32_t type;
		float angular;
		float forward;
	} AngularForwardCommand;

/* Packet for sending breadcrumbs */
	typedef struct WaypointCommand_t {
		uint32_t type;
		float x;
		float y;
		float heading;
		float max_speed;
 	} WaypointCommand;

/*Packet for vector paths*/
	typedef struct VectorCommand_t {
		uint32_t type;
		float x1;
		float y1;
		float x2;
		float y2;
		float max_speed;
	} VectorCommand;

/*Packet for reflexive halt*/
	typedef struct StopCommand_t {
		uint32_t type;
		uint32_t force;
	} StopCommand; 

/*Relatively useless command - actually throws the estop, kept for legacy*/
	typedef struct EStopCommand_t {
		uint32_t type;
	} EStopCommand;

/*Packet to resume from reflexive halt*/
	typedef struct ResumeCommand_t { 
		uint32_t type;
	} ResumeCommand;

/*Packet to tweak our pso*/
	typedef struct PSOTweak_t {
		uint32_t type;
		float deltaX;
		float deltaY;
		float deltaTheta;
	} PSOTweakCommand; 

/*Packet to ack numerous wonderful things with*/
	typedef struct Ack_t {
		uint32_t type;
		uint32_t value;
	} Ack;

/*Packet to send out all kinds of fun info (like battery voltage*/
	typedef struct SysInfo_t {
		uint32_t type;
		float voltage;
	} SysInfoPacket;

#ifdef	__cplusplus
}
#endif

#endif	/* _PACKETS_H */

