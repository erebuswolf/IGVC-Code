/*
 * File:   packets.h
 * Author: Eric Perko
 *
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
	 *
	 * TODO switch these to enums
	 */
	#define HEADINGSPEEDCOMMAND_t 1
	#define REBOOTCOMMAND_t 0
	#define WAYPOINTCOMMAND_t 2
	#define STARTCOMMAND_t 3
	#define STOPCOMMAND_t 4
	#define QUERYSYSTEMSTATUS_t 5
	#define ESTOP_t 6
	#define ANGULARRATESPEEDCOMMAND_t 7

	#define POSE_t 0
	#define DIAGNOSTICS_t 1
	#define SONARS_t 2

	typedef struct poseToBroadcast {
		int8_t type;
		int8_t data1; /* padding */
		int8_t data2;
		int8_t data3;
		float x;
		float y;
		float theta;
		float vel;
		float omega;
		float x_variance;
		float y_variance;
		float theta_variance;
		float vel_variance;
		float omega_variance;
		float front_sonar_ping;
	} Pose;

	typedef struct sonarToBroadcast {
		int8_t type;
		int8_t data1; /* padding */
		int8_t data2;
		int8_t data3;
		float front_sonar;
	} Sonars;
/*
	typedef struct diagnosticsPacket_t {
		int8_t type;
		int8_t status;
		int8_t numDiagnostics;
		int8_t data1; //padding
		HarlieStatus * statuses;
	} DiagnosticsPacket;
*/
	typedef struct Command_t {
		int8_t type;
		char commandData[BUF_SIZE - sizeof(int8_t)];
	} Command;

	typedef struct HeadingSpeedCommand_t {
		int8_t type;
		int8_t data1; /* the following are for padding purposes */
		int8_t data2;
		int8_t data3;
		float desiredHeading;
		float desiredSpeed;
	} HeadingSpeedCommand;

	typedef struct AngularRateSpeedCommand_t {
		int8_t type;
		int8_t data1; /* the following are for padding purposes */
		int8_t data2;
		int8_t data3;
		float desiredAngularRate;
		float desiredSpeed;
	} AngularRateSpeedCommand;

	typedef struct WaypointCommand_t {
		int8_t type;
		int8_t data1; /* the following are for padding purposes */
		int8_t data2;
		int8_t data3;
		//Point2d p;
		float x;
		float y;
		float heading;
		float max_speed;
	} WaypointCommand;

	typedef struct QuerySystemStatus_t {
		int8_t type;
		int8_t verbose;
	} QuerySystemStatus;

	typedef struct StopCommand_t {
		int8_t type;
		int8_t force;
	} StopCommand;

#ifdef	__cplusplus
}
#endif

#endif	/* _PACKETS_H */

