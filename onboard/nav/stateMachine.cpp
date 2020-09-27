#include "stateMachine.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <map>

#include "rover_msgs/NavStatus.hpp"
#include "utilities.hpp"
#include "search/spiralOutSearch.hpp"
#include "search/spiralInSearch.hpp"
#include "search/lawnMowerSearch.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"
#include "gate_search/diamondGateSearch.hpp"

// Constructs a StateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine( lcm::LCM& lcmObject )
    : mPhoebe( nullptr )
    , mLcmObject( lcmObject )
    , mTotalWaypoints( 0 )
    , mCompletedWaypoints( 0 )
    , mRepeaterDropComplete ( false )
    , mStateChanged( true )
{
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    mPhoebe = new Rover( mRoverConfig, lcmObject );
    mSearchStateMachine = SearchFactory( this, SearchType::SPIRALOUT );
    mGateStateMachine = GateFactory( this, mPhoebe, mRoverConfig );
    mObstacleAvoidanceStateMachine = ObstacleAvoiderFactory( this, ObstacleAvoidanceAlgorithm::SimpleAvoidance );
} // StateMachine()

// Destructs the StateMachine object. Deallocates memory for the Rover
// object.
StateMachine::~StateMachine( )
{
    delete mPhoebe;
}

void StateMachine::setSearcher( SearchType type )
{
    assert( mSearchStateMachine );
    delete mSearchStateMachine;
    mSearchStateMachine = SearchFactory( this, type );
}

void StateMachine::updateCompletedPoints( )
{
    mCompletedWaypoints += 1;
    return;
}

void StateMachine::updateRepeaterComplete( )
{
    mRepeaterDropComplete = true;
    return;
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleAngle( double bearing )
{
    mObstacleAvoidanceStateMachine->updateObstacleAngle( bearing );
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleDistance( double distance )
{
    mObstacleAvoidanceStateMachine->updateObstacleDistance( distance );
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleElements( double bearing, double distance )
{
    updateObstacleAngle( bearing );
    updateObstacleDistance( distance );
}

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run()
{
    publishNavState();
    if( isRoverReady() )
    {
        mStateChanged = false;
        NavState nextState = NavState::Unknown;

        if( !mPhoebe->roverStatus().autonState().is_auton )
        {
            nextState = NavState::Off;
            mPhoebe->roverStatus().currentState() = executeOff(); // turn off immediately
            clear( mPhoebe->roverStatus().path() );
            if( nextState != mPhoebe->roverStatus().currentState() )
            {
                mPhoebe->roverStatus().currentState() = nextState;
                mStateChanged = true;
            }
            return;
        }
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

            case NavState::RadioRepeaterTurn:
            case NavState::Turn:
            {
                nextState = executeTurn();
                break;
            }

            case NavState::RadioRepeaterDrive:
            case NavState::Drive:
            {
                nextState = executeDrive();
                break;
            }

            case NavState::RepeaterDropWait:
            {
                nextState = executeRepeaterDropWait();
                break;
            }

            case NavState::SearchFaceNorth:
            case NavState::SearchSpin:
            case NavState::SearchSpinWait:
            case NavState::SearchTurn:
            case NavState::SearchDrive:
            case NavState::TurnToTarget:
            case NavState::TurnedToTargetWait:
            case NavState::DriveToTarget:
            {
                nextState = mSearchStateMachine->run( mPhoebe, mRoverConfig );
                break;
            }

            case NavState::TurnAroundObs:
            case NavState::SearchTurnAroundObs:
            case NavState::DriveAroundObs:
            case NavState::SearchDriveAroundObs:
            {
                nextState = mObstacleAvoidanceStateMachine->run( mPhoebe, mRoverConfig );
                break;
            }

            case NavState::ChangeSearchAlg:
            {
                static int searchFails = 0;
                static double visionDistance = mRoverConfig[ "computerVision" ][ "visionDistance" ].GetDouble();

                switch( mRoverConfig[ "search" ][ "order" ][ searchFails % mRoverConfig[ "search" ][ "numSearches" ].GetInt() ].GetInt() )
                {
                    case 0:
                    {
                        setSearcher(SearchType::SPIRALOUT);
                        break;
                    }
                    case 1:
                    {
                        setSearcher(SearchType::LAWNMOWER);
                        break;
                    }
                    case 2:
                    {
                        setSearcher(SearchType::SPIRALIN);
                        break;
                    }
                    default:
                    {
                        setSearcher(SearchType::SPIRALOUT);
                        break;
                    }
                }
                mSearchStateMachine->initializeSearch( mPhoebe, mRoverConfig, visionDistance );
                if( searchFails % 2 == 1 && visionDistance > 0.5 )
                {
                    visionDistance *= 0.5;
                }
                searchFails += 1;
                nextState = NavState::SearchTurn;
                break;
            }

            case NavState::GateSpin:
            case NavState::GateSpinWait:
            case NavState::GateTurn:
            case NavState::GateDrive:
            case NavState::GateTurnToCentPoint:
            case NavState::GateDriveToCentPoint:
            case NavState::GateFace:
            case NavState::GateShimmy:
            case NavState::GateDriveThrough:
            {
                nextState = mGateStateMachine->run();
                break;
            }

            case NavState::Unknown:
            {
                cerr << "Entered unknown state.\n";
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
        cerr << flush;
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

// Updates the target information of the rover's status.
void StateMachine::updateRoverStatus( TargetList targetList )
{
    Target target1 = targetList.targetList[0];
    Target target2 = targetList.targetList[1];
    mNewRoverStatus.target() = target1;
    mNewRoverStatus.target2() = target2;
} // updateRoverStatus( Target )

// Updates the radio signal strength information of the rover's status.
void StateMachine::updateRoverStatus( RadioSignalStrength radioSignalStrength )
{
    mNewRoverStatus.radio() = radioSignalStrength;
} // updateRoverStatus( RadioSignalStrength )

// Return true if we want to execute a loop in the state machine, false
// otherwise.
bool StateMachine::isRoverReady() const
{
    return mStateChanged || // internal data has changed
           mPhoebe->updateRover( mNewRoverStatus ) || // external data has changed
           mPhoebe->roverStatus().currentState() == NavState::SearchSpinWait || // continue even if no data has changed
           mPhoebe->roverStatus().currentState() == NavState::TurnedToTargetWait || // continue even if no data has changed
           mPhoebe->roverStatus().currentState() == NavState::RepeaterDropWait ||
           mPhoebe->roverStatus().currentState() == NavState::GateSpinWait;

} // isRoverReady()

// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const
{
    NavStatus navStatus;
    navStatus.nav_state_name = stringifyNavState();
    navStatus.completed_wps = mCompletedWaypoints;
    navStatus.total_wps = mTotalWaypoints;
    const string& navStatusChannel = mRoverConfig[ "lcmChannels" ][ "navStatusChannel" ].GetString();
    mLcmObject.publish( navStatusChannel, &navStatus );
} // publishNavState()

// Executes the logic for off. If the rover is turned on, it updates
// the roverStatus. If the course is empty, the rover is done  with
// the course otherwise it will turn to the first waypoing. Else the
// rover is still off.
NavState StateMachine::executeOff()
{
    if( mPhoebe->roverStatus().autonState().is_auton )
    {
        mCompletedWaypoints = 0;
        mTotalWaypoints = mPhoebe->roverStatus().course().num_waypoints;

        if( !mTotalWaypoints )
        {
            return NavState::Done;
        }
        return NavState::Turn;
    }
    mPhoebe->stop();
    return NavState::Off;
} // executeOff()

// Executes the logic for the done state. Stops and turns off the
// rover.
NavState StateMachine::executeDone()
{
    mPhoebe->stop();
    return NavState::Done;
} // executeDone()

// Executes the logic for the turning. If the rover is turned off, it
// proceeds to Off. If the rover finishes turning, it drives to the
// next Waypoint. Else the rover keeps turning to the Waypoint.
NavState StateMachine::executeTurn()
{
    if( mPhoebe->roverStatus().path().empty() )
    {
        return NavState::Done;
    }
    // If we should drop a repeater and have not already, add last
    // point where connection was good to front of path and turn
    // if ( isAddRepeaterDropPoint() )
    // {
    //     addRepeaterDropPoint();
    //     return NavState::RadioRepeaterTurn;
    // }

    Odometry& nextPoint = mPhoebe->roverStatus().path().front().odom;
    if( mPhoebe->turn( nextPoint ) )
    {
        if (mPhoebe->roverStatus().currentState() == NavState::RadioRepeaterTurn)
        {
            return NavState::RadioRepeaterDrive;
        }
        return NavState::Drive;
    }

    if (mPhoebe->roverStatus().currentState() == NavState::RadioRepeaterTurn)
    {
        return NavState::RadioRepeaterTurn;
    }
    return NavState::Turn;
} // executeTurn()

// Executes the logic for driving. If the rover is turned off, it
// proceeds to Off. If the rover finishes driving, it either starts
// searching for a target (dependent the search parameter of
// the Waypoint) or it turns to the next Waypoint. If the rover
// detects an obstacle, it goes to turn around it. Else the rover
// keeps driving to the next Waypoint.
NavState StateMachine::executeDrive()
{
    const Waypoint& nextWaypoint = mPhoebe->roverStatus().path().front();
    double distance = estimateNoneuclid( mPhoebe->roverStatus().odometry(), nextWaypoint.odom );

    // If we should drop a repeater and have not already, add last
    // point where connection was good to front of path and turn
    // if ( isAddRepeaterDropPoint() )
    // {
    //     addRepeaterDropPoint();
    //     return NavState::RadioRepeaterTurn;
    // }

    if( isObstacleDetected() && !isWaypointReachable( distance ) )
    {
        mObstacleAvoidanceStateMachine->updateObstacleElements( getOptimalAvoidanceAngle(),
                                                                getOptimalAvoidanceDistance() );
        return NavState::TurnAroundObs;
    }
    DriveStatus driveStatus = mPhoebe->drive( nextWaypoint.odom );
    if( driveStatus == DriveStatus::Arrived )
    {
        if( nextWaypoint.search )
        {
            return NavState::SearchSpin;
        }
        mPhoebe->roverStatus().path().pop_front();
        if (mPhoebe->roverStatus().currentState() == NavState::RadioRepeaterDrive)
        {
            return NavState::RepeaterDropWait;
        }
        ++mCompletedWaypoints;
        return NavState::Turn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        if (mPhoebe->roverStatus().currentState() == NavState::RadioRepeaterDrive)
        {
            return NavState::RadioRepeaterDrive;
        }
        return NavState::Drive;
    }
    // else driveStatus == DriveStatus::OffCourse (must turn to waypoint)
    if (mPhoebe->roverStatus().currentState() == NavState::RadioRepeaterDrive)
    {
        return NavState::RadioRepeaterTurn;
    }

    return NavState::Turn;
} // executeDrive()

// Executes the logic for waiting during a radio repeater drop
// If the rover is done waiting, it continues the original course.
// Else the rover keeps waiting.
NavState StateMachine::executeRepeaterDropWait( )
{

    RepeaterDropInit rr_init;
    const string& radioRepeaterInitChannel = mRoverConfig[ "lcmChannels" ][ "repeaterDropInitChannel" ].GetString();
    mLcmObject.publish( radioRepeaterInitChannel, &rr_init );

    if( mRepeaterDropComplete )
    {
        return NavState::Turn;
    }
    return NavState::RepeaterDropWait;
}

// Gets the string representation of a nav state.
string StateMachine::stringifyNavState() const
{
    static const map<NavState, std::string> navStateNames =
        {
            { NavState::Off, "Off" },
            { NavState::Done, "Done" },
            { NavState::Turn, "Turn" },
            { NavState::Drive, "Drive" },
            { NavState::SearchFaceNorth, "Search Face North" },
            { NavState::SearchSpin, "Search Spin" },
            { NavState::SearchSpinWait, "Search Spin Wait" },
            { NavState::ChangeSearchAlg, "Change Search Algorithm" },
            { NavState::SearchTurn, "Search Turn" },
            { NavState::SearchDrive, "Search Drive" },
            { NavState::TurnToTarget, "Turn to Target" },
            { NavState::TurnedToTargetWait, "Turned to Target Wait" },
            { NavState::DriveToTarget, "Drive to Target" },
            { NavState::TurnAroundObs, "Turn Around Obstacle"},
            { NavState::DriveAroundObs, "Drive Around Obstacle" },
            { NavState::SearchTurnAroundObs, "Search Turn Around Obstacle" },
            { NavState::SearchDriveAroundObs, "Search Drive Around Obstacle" },
            { NavState::GateSpin, "Gate Spin" },
            { NavState::GateSpinWait, "Gate Spin Wait" },
            { NavState::GateTurn, "Gate Turn" },
            { NavState::GateDrive, "Gate Drive" },
            { NavState::GateTurnToCentPoint, "Gate Turn to Center Point" },
            { NavState::GateDriveToCentPoint, "Gate Drive to Center Point" },
            { NavState::GateFace, "Gate Face" },
            { NavState::GateShimmy, "Gate Shimmy" },
            { NavState::GateDriveThrough, "Gate Drive Through" },
            { NavState::RadioRepeaterTurn, "Radio Repeater Turn" },
            { NavState::RadioRepeaterDrive, "Radio Repeater Drive" },
            { NavState::RepeaterDropWait, "Radio Repeater Drop" },
            { NavState::Unknown, "Unknown" }
        };

    return navStateNames.at( mPhoebe->roverStatus().currentState() );
} // stringifyNavState()

// Returns true if an obstacle is detected, false otherwise.
bool StateMachine::isObstacleDetected() const
{
    return mPhoebe->roverStatus().obstacle().detected;
} // isObstacleDetected()

// Returns the optimal angle to avoid the detected obstacle.
double StateMachine::getOptimalAvoidanceAngle() const
{
    return mPhoebe->roverStatus().obstacle().bearing;
} // optimalAvoidanceAngle()

// Returns the optimal angle to avoid the detected obstacle.
double StateMachine::getOptimalAvoidanceDistance() const
{
    return mPhoebe->roverStatus().obstacle().distance + mRoverConfig[ "navThresholds" ][ "waypointDistance" ].GetDouble();
} // optimalAvoidanceAngle()

bool StateMachine::isWaypointReachable( double distance )
{
    return isLocationReachable( mPhoebe, mRoverConfig, distance, mRoverConfig["navThresholds"]["waypointDistance"].GetDouble());
} // isWaypointReachable

// If we have not already begun to drop radio repeater
//  (RadioRepeaterTurn or RadioRepeaterDrive)
// We should drop the repeater
// and we haven't already dropped one
bool StateMachine::isAddRepeaterDropPoint() const
{

    return ( mPhoebe->roverStatus().currentState() != NavState::RadioRepeaterTurn &&
             mPhoebe->roverStatus().currentState() != NavState::RadioRepeaterDrive &&
             mPhoebe->isTimeToDropRepeater() &&
             mRepeaterDropComplete == false );
} //isAddRepeaterDropPoint

// Returns whether or not to enter RadioRepeaterTurn state.
void StateMachine::addRepeaterDropPoint()
{
    // TODO: signal drops before first completed waypoint
    // Get last waypoint in path (where signal was good) and set search and gate
    // to false in order to not repeat search
    Waypoint way = (mPhoebe->roverStatus().course().waypoints)[mCompletedWaypoints-1];
    way.search = false;
    way.gate = false;

    mPhoebe->roverStatus().path().push_front(way);
} // addRepeaterDropPoint


// TODOS:
// [drive to target] obstacle and target
// all of code, what to do in cases of both target and obstacle
// thresholds based on state? waypoint vs target
