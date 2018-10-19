#ifndef ROVER_HPP
#define ROVER_HPP

#include <lcm/lcm-cpp.hpp>
#include <queue>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/TennisBall.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "rapidjson/document.h"
#include "pid.hpp"

using namespace rover_msgs;
using namespace std;

// This class is the representation of the navigation states.
enum class NavState
{
	Off = 0,
	Done = 1,
	Turn = 10,
	Drive = 11,
	SearchFaceNorth = 20,
	SearchTurn120 = 21,
	SearchTurn240 = 22,
	SearchTurn360 = 23,
	SearchTurn = 24,
	SearchDrive = 25,
	TurnToBall = 26,
	DriveToBall = 27,
	TurnAroundObs = 30,
	DriveAroundObs = 31,
	SearchTurnAroundObs = 32,
	SearchDriveAroundObs = 33,
	Unknown = 255
}; // AutonState

// This class is the representation of the drive status.
enum class DriveStatus
{
	Arrived,
	OnCourse,
	OffCourse
}; // DriveStatus

// This class creates a Rover object which can perform operations that
// the real rover can perform.
class Rover
{
public:
	// This class holds all the status informatin of the rover.
	class RoverStatus
	{
	public:
		RoverStatus();

		RoverStatus(
			NavState navState,
			AutonState autonStateIn,
			Course courseIn,
			Obstacle obstacleIn,
			Odometry odometryIn,
			TennisBall tennisBallIn
			);

		NavState& currentState();

		AutonState& autonState();

		Course& course();

		queue<Waypoint>& path();

		Obstacle& obstacle();

		Odometry& odometry();

		TennisBall& tennisBall();

		RoverStatus& operator=( RoverStatus& newRoverStatus );

	private:
		// The rover's current navigation state.
		NavState mCurrentState;

		// The rover's current auton state.
		AutonState mAutonState;

		// The rover's overall course.
		Course mCourse;

		// The rover's current path. The path is initially the same as
		// the rover's course, however, as waypoints are visited, the
		// are removed from the path but not the course.
		queue<Waypoint> mPath;

		// The rover's current obstacle information from computer
		// vision.
		Obstacle mObstacle;

		// The rover's current odometry information.
		Odometry mOdometry;

		// The rover's current tennis ball information from computer
		// vision.
		TennisBall mTennisBall;
	};

	Rover( const rapidjson::Document& config, lcm::LCM& lcm_in );

	DriveStatus drive( const Odometry& destination );

	DriveStatus drive( const double distance, const double bearing );

	bool turn( Odometry& destination );

	bool turn( double bearing );

	void stop();

	bool updateRover( RoverStatus newRoverStatus );

	RoverStatus& roverStatus();

	PidLoop& distancePid();

	PidLoop& bearingPid();

	const double longMeterInMinutes() const;

private:
	/*************************************************************************/
	/* Private Member Functions */
	/*************************************************************************/
	void publishJoystick( const double forwardBack, const double leftRight, const bool kill );

	bool isEqual( const Obstacle& obstacle1, const Obstacle& obstacle2 ) const;

	bool isEqual( const Odometry& odometry1, const Odometry& odometry2 ) const;

	bool isEqual( const TennisBall& tennisBall1, const TennisBall& tennisBall2 ) const;

	/*************************************************************************/
	/* Private Member Variables */
	/*************************************************************************/

	// The rover's current status.
	RoverStatus mRoverStatus;

	// A reference to the configuration file.
	const rapidjson::Document& mRoverConfig;

	// A reference to the lcm object that will be used for
	// communicating with the actual rover and the base station.
	lcm::LCM& mLcmObject;

	// The pid loop for driving.
	PidLoop mDistancePid;

	// The pid loop for turning.
	PidLoop mBearingPid;

	// The conversion factor from arcminutes to meters. This is based
	// on the rover's current latitude.
	double mLongMeterInMinutes;
};

#endif // ROVER_HPP
