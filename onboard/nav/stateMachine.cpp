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

// Constructs a StateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine( lcm::LCM& lcmObject )
    : mPhoebe( nullptr )
    , mLcmObject( lcmObject )
    , mTotalWaypoints( 0 )
    , mCompletedWaypoints( 0 )
    , mMissedWaypoints( 0 )
    , mFoundTargets( 0 )
    , mTotalTargets( 0 )
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

void StateMachine::updateFoundTargets( )
{
    mFoundTargets += 1;
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
    if( isRoverReady() )
    {
        publishNavState();
        mStateChanged = false;
        NavState nextState = NavState::Unknown;

        if( !mPhoebe->roverStatus().autonState().is_auton )
        {
            nextState = NavState::Off;
            mPhoebe->roverStatus().currentState() = executeOff(); // turn off immediately
            publishNavState();
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
        cout << flush;
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

// Return true if we want to execute a loop in the state machine, false
// otherwise.
bool StateMachine::isRoverReady() const
{
    return mStateChanged || // internal data has changed
           mPhoebe->updateRover( mNewRoverStatus ) || // external data has changed
           mPhoebe->roverStatus().currentState() == NavState::SearchSpinWait || // continue even if no data has changed
           mPhoebe->roverStatus().currentState() == NavState::TurnedToTargetWait; // continue even if no data has changed
} // isRoverReady()

// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const
{
    NavStatus navStatus;
    navStatus.nav_state = static_cast<int8_t>( mPhoebe->roverStatus().currentState() );
    navStatus.nav_state_name = stringifyNavState();
    navStatus.completed_wps = mCompletedWaypoints;
    navStatus.missed_wps = mMissedWaypoints;
    navStatus.total_wps = mTotalWaypoints;
    navStatus.found_tbs = mFoundTargets;
    navStatus.total_tbs = mTotalTargets;
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
        mMissedWaypoints = 0;
        mFoundTargets = 0;
        mTotalWaypoints = mPhoebe->roverStatus().course().num_waypoints;
        mTotalTargets = mPhoebe->roverStatus().getPathTargets();

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

    Odometry& nextPoint = mPhoebe->roverStatus().path().front().odom;
    if( mPhoebe->turn( nextPoint ) )
    {
        return NavState::Drive;
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

    if( isObstacleDetected() && !isWaypointReachable( distance ) )
    {
        mObstacleAvoidanceStateMachine->updateObstacleElements( getOptimalAvoidanceAngle(), getOptimalAvoidanceDistance() );
        return NavState::TurnAroundObs;
    }
    DriveStatus driveStatus = mPhoebe->drive( nextWaypoint.odom );
    if( driveStatus == DriveStatus::Arrived )
    {
        if( nextWaypoint.search )
        {
            return NavState::SearchSpin;
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

// TODOS:
// [drive to target] obstacle and target
// all of code, what to do in cases of both target and obstacle
// thresholds based on state? waypoint vs target
