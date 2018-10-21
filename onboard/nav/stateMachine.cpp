#include "stateMachine.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

#include "rover_msgs/NavStatus.hpp"
#include "utilities.hpp"

// Constructs a StateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine( lcm::LCM& lcmObject )
	: mPhoebe( nullptr )
	, mLcmObject( lcmObject )
	, mOriginalObstacleAngle( 0 )
	, mTotalWaypoints( 0 )
	, mCompletedWaypoints( 0 )
	, mMissedWaypoints( 0 )
	, mStateChanged( true )
{
	ifstream configFile;
	configFile.open( "/vagrant/onboard/nav/config.json" );
	string config = "";
	string setting;
	while( configFile >> setting )
	{
		config += setting;
	}
	configFile.close();
	mRoverConfig.Parse( config.c_str() );
	mPhoebe = new Rover( mRoverConfig, lcmObject );
} // StateMachine()

// Destructs the StateMachine object. Deallocates memory for the Rover
// object.
StateMachine::~StateMachine()
{
	delete mPhoebe;
}

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run()
{
	if( mStateChanged || mPhoebe->updateRover( mNewRoverStatus ) )
	{
		publishNavState();
		mStateChanged = false;
		NavState nextState = NavState::Unknown;

		switch( mPhoebe->roverStatus().currentState() )
		{
			case NavState::Off:
			{
				nextState = executeOff();
				break;
			}

			case NavState::Done:
			{
				nextState = executeDone();
				break;
			}

			case NavState::Turn:
			{
				nextState = executeTurn();
				break;
			}

			case NavState::Drive:
			{
				nextState = executeDrive();
				break;
			}

			case NavState::SearchFaceNorth:
			{
				nextState = executeSearchFaceNorth();
				break;
			}

			case NavState::SearchTurn120:
			{
				nextState = executeSearchTurn120();
				break;
			}

			case NavState::SearchTurn240:
			{
				nextState = executeSearchTurn240();
				break;
			}

			case NavState::SearchTurn360:
			{
				nextState = executeSearchTurn360();
				break;
			}

			case NavState::SearchTurn:
			{
				nextState = executeSearchTurn();
				break;
			}

			case NavState::SearchDrive:
			{
				nextState = executeSearchDrive();
				break;
			}

			case NavState::TurnToBall:
			{
				nextState = executeTurnToBall();
				break;
			}

			case NavState::DriveToBall:
			{
				nextState = executeDriveToBall();
				break;
			}

			case NavState::TurnAroundObs:
			case NavState::SearchTurnAroundObs:
			{
				nextState = executeTurnAroundObs();
				break;
			}

			case NavState::DriveAroundObs:
			case NavState::SearchDriveAroundObs:
			{
				nextState = executeDriveAroundObs();
				break;
			}
			case NavState::Unknown:
			{
				cout << "Entered unknown state.\n";
				exit(1);
			}
		} // switch

		if( nextState != mPhoebe->roverStatus().currentState() )
		{
			mStateChanged = true;
			mPhoebe->roverStatus().currentState() = nextState;
			mPhoebe->distancePid().reset();
			mPhoebe->bearingPid().reset();
		}
	} // if
} // run()

// Updates the auton state (on/off) of the rover's status.
void StateMachine::updateRoverStatus( AutonState autonState )
{
	mNewRoverStatus.autonState() = autonState;
} // updateRoverStatus( AutonState )

// Updates the course of the rover's status if it has changed.
void StateMachine::updateRoverStatus( Course course )
{
	if( mNewRoverStatus.course().hash != course.hash )
	{
		mNewRoverStatus.course() = course;
	}
} // updateRoverStatus( Course )

// Updates the obstacle information of the rover's status.
void StateMachine::updateRoverStatus( Obstacle obstacle )
{
	mNewRoverStatus.obstacle() = obstacle;
} // updateRoverStatus( Obstacle )

// Updates the odometry information of the rover's status.
void StateMachine::updateRoverStatus( Odometry odometry )
{
	mNewRoverStatus.odometry() = odometry;
} // updateRoverStatus( Odometry )

// Updates the tennis ball information of the rover's status.
void StateMachine::updateRoverStatus( TennisBall tennisBall )
{
	mNewRoverStatus.tennisBall() = tennisBall;
} // updateRoverStatus( TennisBall )

// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const
{
	NavStatus navStatus;
	navStatus.nav_state = static_cast<int8_t>( mPhoebe->roverStatus().currentState() );
	navStatus.completed_wps = mCompletedWaypoints;
	navStatus.missed_wps = mMissedWaypoints;
	navStatus.total_wps = mTotalWaypoints;
	const string& navStatusChannel = mRoverConfig[ "navStatusChannel" ].GetString();
	mLcmObject.publish( navStatusChannel, &navStatus );
} // publishNavState()

// void StateMachine::printNavState() const
// {
// 	// switch
// }

// Executes the logic for off. If the rover is turned on, it updates
// the roverStatus. If the course is empty, the rover is done  with
// the course otherwise it will turn to the first waypoing. Else the
// rover is still off.
NavState StateMachine::executeOff()
{
	if( mPhoebe->roverStatus().autonState().is_auton )
	{
		mCompletedWaypoints = 0;
		mMissedWaypoints = 0;
		mTotalWaypoints = mPhoebe->roverStatus().course().num_waypoints;

		if( !mTotalWaypoints )
		{
			return NavState::Done;
		}
		return NavState::Turn;
	}
	return NavState::Off;
} // executeOff()

// Executes the logic for the done state. Stops and turns off the
// rover.
NavState StateMachine::executeDone()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	mPhoebe->stop();
	return NavState::Done;
} // executeDone()

// Executes the logic for the turning. If the rover is turned off, it
// proceeds to Off. If the rover finishes turning, it drives to the
// next Waypoint. Else the rover keeps turning to the Waypoint.
NavState StateMachine::executeTurn()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().path().empty() )
	{
		return NavState::Done;
	}

	Odometry& nextPoint = mPhoebe->roverStatus().path().front().odom;
	if( mPhoebe->turn( nextPoint ) )
	{
		return NavState::Drive;
	}
	return NavState::Turn;
} // executeTurn()

// Executes the logic for driving. If the rover is turned off, it
// proceeds to Off. If the rover finishes driving, it either starts
// searching for a tennis ball (dependent the search parameter of
// the Waypoint) or it turns to the next Waypoint. If the rover
// detects an obstacle, it goes to turn around it. Else the rover
// keeps driving to the next Waypoint.
NavState StateMachine::executeDrive()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	// if( mPhoebe->roverStatus().path().empty() )
	// {
	// 	return NavState::Done;
	// }
	if( mPhoebe->roverStatus().obstacle().detected )
	{
		mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
		return NavState::TurnAroundObs;
	}

	const Waypoint& nextWaypoint = mPhoebe->roverStatus().path().front();
	DriveStatus driveStatus = mPhoebe->drive( nextWaypoint.odom );
	if( driveStatus == DriveStatus::Arrived )
	{
		if( nextWaypoint.search )
		{
			return NavState::SearchFaceNorth;
		}
		mPhoebe->roverStatus().path().pop();
		++mCompletedWaypoints;
		return NavState::Turn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
		return NavState::Drive;
	}
	// if driveStatus == DriveStatus::OffCourse
	return NavState::Turn;
} // executeDrive()

// Executes the logic for turning to face north to orient itself for
// a search. If the rover is turned off, it proceeds to Off. If the
// rover detects the tennis ball, it proceeds to the ball If the rover
// finishes turning, it proceeds to SearchTurn120. Else the rover keeps
// turning to north.
NavState StateMachine::executeSearchFaceNorth()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 90 ) )
	{
		return NavState::SearchTurn120;
	}
	return NavState::SearchFaceNorth;
} // executeSearchFaceNorth

// Executes the logic for the first third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchTurn240.
// Else the rover keeps turning to 120 degrees.
NavState StateMachine::executeSearchTurn120()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 210 ) )
	{
		return NavState::SearchTurn240;
	}
	return NavState::SearchTurn120;
} // executeSearchTurn120()

// Executes the logic for the second third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchTurn360.
// Else the rover keeps turning to 240 degrees.
NavState StateMachine::executeSearchTurn240()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 330 ) )
	{
		return NavState::SearchTurn360;
	}
	return NavState::SearchTurn240;
} // executeSearchTurn240

// Executes the logic for the final third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, the next state is SearchDrive.
// Else the rover keeps turning to 360 degrees.
NavState StateMachine::executeSearchTurn360()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 90 ) )
	{
		initializeSearch();
		return NavState::SearchTurn;
	}
	return NavState::SearchTurn360;
} // executeSearchTurn360()

// Executes the logic for turning while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes turning,
// it proceeds to driving while searching. Else the rover keeps
// turning to the next Waypoint.
NavState StateMachine::executeSearchTurn()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mSearchPoints.empty() )
	{
		if( !addFourPointsToSearch() )
		{
			mPhoebe->roverStatus().path().pop();
			++mMissedWaypoints;
			return NavState::Turn;
		}
		// return NavState::SearchTurn;
	}

	Odometry& nextSearchPoint = mSearchPoints.front();
	if( mPhoebe->turn( nextSearchPoint ) )
	{
		return NavState::SearchDrive;
	}
	return NavState::SearchTurn;
} // executeSearchTurn()

// Executes the logic for driving while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes driving,
// it proceeds to turning to the next Waypoint. If the rover detects
// an obstacle, it proceeds to obstacle avoidance. Else the rover
// keeps driving to the next Waypoint.
NavState StateMachine::executeSearchDrive()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mPhoebe->roverStatus().obstacle().detected )
	{
		mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
		return NavState::SearchTurnAroundObs;
	}

	const Odometry& nextSearchPoint = mSearchPoints.front();
	DriveStatus driveStatus = mPhoebe->drive( nextSearchPoint );
	if( driveStatus == DriveStatus::Arrived )
	{
		mSearchPoints.pop();
		return NavState::SearchTurn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
		return NavState::SearchDrive;
	}
	// if driveStatus == DriveStatus::OffCourse
	return NavState::SearchTurn;
} // executeSearchDrive()

// Executes the logic for turning to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts to search again. If the rover finishes turning to the ball,
// it drives to the ball. Else the rover continues to turn to to the
// ball.
NavState StateMachine::executeTurnToBall()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( !mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::SearchFaceNorth;
	}
	if( mPhoebe->turn( mPhoebe->roverStatus().tennisBall().bearing ) )
	{
		return NavState::DriveToBall;
	}
	return NavState::TurnToBall;
} // executeTurnToBall()

// Executes the logic for driving to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts the search again. If the rover detects an obstacle, it
// proceeds to go around the obstacle. If the rover finishes driving
// to the ball, it moves on to the next Waypoint. If the rover gets
// off course, it proceeds to turn back to the Waypoint. Else, it
// continues driving to the ball.
NavState StateMachine::executeDriveToBall()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( !mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::SearchFaceNorth;
	}
	// TODO: save location of ball then go around object?
	if( mPhoebe->roverStatus().obstacle().detected )
	{
		mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
		return NavState::SearchTurnAroundObs;
	}
	DriveStatus driveStatus = mPhoebe->drive( mPhoebe->roverStatus().tennisBall().distance,
											mPhoebe->roverStatus().tennisBall().bearing );
	if( driveStatus == DriveStatus::Arrived )
	{
		mPhoebe->roverStatus().path().pop();
		++mCompletedWaypoints;
		return NavState::Turn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
		return NavState::DriveToBall;
	}
	// if driveStatus == DriveStatus::OffCourse
	return NavState::TurnToBall;
} // executeDriveToBall()

// Executes the logic for turning around an obstacle. If the rover is
// turned off, it proceeds to Off. If the tennis ball is detected, the
// rover proceeds to it. If the Waypopint and obstacle are in similar
// locations, assume that we would have seen the ball and move on. If
// the obstacle is no longer detcted, proceed to drive around the
// obstacle. Else continue turning around the obstacle.
// ASSUMPTION: To avoid an infinite loop, we assume that the obstacle is straight ahead of us,
//			   therefore we produce an underestimate for how close the waypoint is to the
//			   obstacle. This relies on using a path width no larger than what we can
//			   confidentally see to the side.
NavState StateMachine::executeTurnAroundObs()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}

	double cvThresh = mRoverConfig[ "cvThresh" ].GetDouble();
	if( ( mPhoebe->roverStatus().currentState() == NavState::TurnAroundObs ) &&
		( estimateNoneuclid( mPhoebe->roverStatus().path().front().odom,
							 mPhoebe->roverStatus().odometry() ) < 2 * cvThresh ) )
	{
		mPhoebe->roverStatus().path().pop();
		++mMissedWaypoints;
		return NavState::Turn;
	}
	if( ( mPhoebe->roverStatus().currentState() == NavState::SearchTurnAroundObs ) &&
		( estimateNoneuclid( mSearchPoints.front(), mPhoebe->roverStatus().odometry() )
		  < 2 * cvThresh ) )
	{
		mSearchPoints.pop();
		return NavState::SearchTurn;
	}
	if( !mPhoebe->roverStatus().obstacle().detected )
	{
		double distanceAroundObs = cvThresh / cos( fabs( degreeToRadian( mOriginalObstacleAngle ) ) );
		mObstacleAvoidancePoint = createAvoidancePoint( distanceAroundObs );
		if( mPhoebe->roverStatus().currentState() == NavState::TurnAroundObs )
		{
			printf("here1\n");
			return NavState::DriveAroundObs;
		}
		return NavState::SearchDriveAroundObs;
	}

	double desiredBearing = mod( mPhoebe->roverStatus().odometry().bearing_deg
							   + mPhoebe->roverStatus().obstacle().bearing, 360 );
	mPhoebe->turn( desiredBearing );
	return mPhoebe->roverStatus().currentState();
} // executeTurnAroundObs()

// Executes the logic for driving around an obstacle. If the rover is
// turned off, proceed to Off. If another obstacle is detected, proceed
// to go around it. If the rover finished going around the obstacle, it
// proceeds to turning to the Waypoint that was being driven to when the
// obstacle was spotted. Else, continue driving around the obstacle.
// TODO: fix the case about when the obstacle gets off course.
NavState StateMachine::executeDriveAroundObs()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().obstacle().detected )
	{
		mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
		if( mPhoebe->roverStatus().currentState() == NavState::DriveAroundObs )
		{
			printf("here2\n");
			return NavState::TurnAroundObs;
		}
		return NavState::SearchTurnAroundObs;
	}

	DriveStatus driveStatus = mPhoebe->drive( mObstacleAvoidancePoint );
	if( driveStatus == DriveStatus::Arrived )
	{
		if( mPhoebe->roverStatus().currentState() == NavState::DriveAroundObs )
		{
			return NavState::Turn;
		}
		return NavState::SearchTurn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
		return mPhoebe->roverStatus().currentState();
	}
	// if driveStatus == DriveStatus::OffCourse
	if( mPhoebe->roverStatus().currentState() == NavState::DriveAroundObs )
	{
		return NavState::TurnAroundObs;
	}
	return NavState::SearchTurnAroundObs;
} // executeDriveAroundObs()

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void StateMachine::initializeSearch()
{
	clear( mSearchPoints );
	mSearchPointMultipliers.clear();
	mSearchPointMultipliers.push_back( pair<short, short> ( 0, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( 1, -1 ) );
	addFourPointsToSearch();
} // initializeSearch()

// true indicates to added search points

// Add the next loop to the search. If the points are added to the
// search, returns true. If the rover is further away from the start
// of the search than the search bail threshold, return false.
bool StateMachine::addFourPointsToSearch()
{
	const double pathWidth = mRoverConfig[ "pathWidth" ].GetDouble();
	if( mSearchPointMultipliers[ 0 ].second * pathWidth > mRoverConfig[ "searchBailThresh" ].GetDouble() )
	{
		return false;
	}

	for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
	{
		Odometry nextSearchPoint = mPhoebe->roverStatus().path().front().odom;
		double totalLatitudeMinutes = nextSearchPoint.latitude_min +
			( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
		double totalLongitudeMinutes = nextSearchPoint.longitude_min +
			( mSearchPointMultiplier.second * pathWidth * mPhoebe->longMeterInMinutes() );

		nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
		// nextSearchPoint.latitude_min += mod( totalLatitudeMinutes, 60 );
		// printf("%f\n", nextSearchPoint.latitude_min);
		nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
		// printf("%f\n", nextSearchPoint.latitude_min);
		nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
		// nextSearchPoint.longitude_min += mod( totalLongitudeMinutes, 60 );
		nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

		mSearchPoints.push( nextSearchPoint );

		mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
		mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
	}
	return true;
} // addFourPointsToSearch()

// Creates the odometry point to use to drive around in obstacle
// avoidance.
Odometry StateMachine::createAvoidancePoint( const double distance )
{
	Odometry avoidancePoint = mPhoebe->roverStatus().odometry();
	double totalLatitudeMinutes = avoidancePoint.latitude_min +
		cos( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * LAT_METER_IN_MINUTES;
	double totalLongitudeMinutes = avoidancePoint.longitude_min +
		sin( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * mPhoebe->longMeterInMinutes();
	avoidancePoint.latitude_deg += totalLatitudeMinutes / 60;
	// avoidancePoint.latitude_min = mod( totalLatitudeMinutes, 60 );
	avoidancePoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes) / 60 ) * 60 );
	avoidancePoint.longitude_deg += totalLongitudeMinutes / 60;
	// avoidancePoint.longitude_min = mod( totalLatitudeMinutes, 60 );
	avoidancePoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );
	return avoidancePoint;
}

// thresholds based on state? waypoint vs ball
// set distance to go around obstacles?
// set threshold for when to skip a point?

// TODOS:
// [turn to ball | drive to ball] if ball lost, restart search a better way??
// [add four points to search] look into this
// look into thresholds for searching
// [drive to ball] obstacle and ball
// all of code, what to do in cases of both ball and obstacle
