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
#include "searches.hpp"

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
    mSearcher = SearchFactory( this, SearchType::SPIRALOUT );
} // StateMachine()

// Destructs the StateMachine object. Deallocates memory for the Rover
// object.
StateMachine::~StateMachine( )
{
    delete mPhoebe;
}

void StateMachine::setSearcher( SearchType type )
{
    assert( mSearcher );
    delete mSearcher;
    mSearcher = SearchFactory( this, type );
}

void StateMachine::updateCompletedPoints( )
{
    mCompletedWaypoints += 1;
    return;
}

void StateMachine::updateObstacleAngle( double angle )
{
    mOriginalObstacleAngle = angle;
    return;
}

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run()
{
    if( isRoverReady() )
    {
        // todo print state, add to publish nav state
        publishNavState();
        mStateChanged = false;
        NavState nextState = NavState::Unknown;

        if( !mPhoebe->roverStatus().autonState().is_auton )
        {
            nextState = NavState::Off;
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
            case NavState::TurnToBall:
            case NavState::DriveToBall:
            {
                nextState = mSearcher->run( mPhoebe, mRoverConfig );
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

            case NavState::ChangeSearchAlg:
            {
                static int searchFails = 0;
                static double pathWidth = mRoverConfig[ "computerVision" ][ "pathWidth" ].GetDouble();

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
                mSearcher->initializeSearch( mPhoebe, mRoverConfig, pathWidth );
                if( searchFails % 2 == 1 && pathWidth > 0.5 )
                {
                    pathWidth *= 0.5;
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

// Updates the tennis ball information of the rover's status.
void StateMachine::updateRoverStatus( TennisBall tennisBall )
{
    mNewRoverStatus.tennisBall() = tennisBall;
} // updateRoverStatus( TennisBall )

// Return true if we want to execute a loop in the state machine, false
// otherwise.
bool StateMachine::isRoverReady() const
{
    return mStateChanged || // internal data has changed
           mPhoebe->updateRover( mNewRoverStatus ) || // external data has changed
           mPhoebe->roverStatus().currentState() == NavState::SearchSpinWait; // continue even if no data has changed
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
// searching for a tennis ball (dependent the search parameter of
// the Waypoint) or it turns to the next Waypoint. If the rover
// detects an obstacle, it goes to turn around it. Else the rover
// keeps driving to the next Waypoint.
NavState StateMachine::executeDrive()
{
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

// Executes the logic for turning around an obstacle. If the rover is
// turned off, it proceeds to Off. If the tennis ball is detected, the
// rover proceeds to it. If the Waypopint and obstacle are in similar
// locations, assume that we would have seen the ball and move on. If
// the obstacle is no longer detcted, proceed to drive around the
// obstacle. Else continue turning around the obstacle.
// ASSUMPTION: To avoid an infinite loop, we assume that the obstacle is straight ahead of us,
//             therefore we produce an underestimate for how close the waypoint is to the
//             obstacle. This relies on using a path width no larger than what we can
//             confidentally see to the side.
NavState StateMachine::executeTurnAroundObs()
{
    if( mPhoebe->roverStatus().tennisBall().found )
    {
        return NavState::TurnToBall;
        // return NavState::Search; // todo
    }

    double cvThresh = mRoverConfig[ "computerVision" ][ "obstacleThresh" ].GetDouble();
    if( ( mPhoebe->roverStatus().currentState() == NavState::TurnAroundObs ) &&
        ( estimateNoneuclid( mPhoebe->roverStatus().path().front().odom,
                             mPhoebe->roverStatus().odometry() ) < 2 * cvThresh ) )
    {
        mPhoebe->roverStatus().path().pop();
        mMissedWaypoints += 1;
        return NavState::Turn;
    }
    if( ( mPhoebe->roverStatus().currentState() == NavState::SearchTurnAroundObs ) &&
        ( estimateNoneuclid( mSearcher->frontSearchPoint(), mPhoebe->roverStatus().odometry() )
          < 2 * cvThresh ) )
    {
        mSearcher->popSearchPoint();
        return NavState::SearchTurn;
        // return NavState::Search; // todo
    }
    if( !mPhoebe->roverStatus().obstacle().detected )
    {
        double distanceAroundObs = cvThresh / cos( fabs( degreeToRadian( mOriginalObstacleAngle ) ) );
        mObstacleAvoidancePoint = createAvoidancePoint( distanceAroundObs );
        if( mPhoebe->roverStatus().currentState() == NavState::TurnAroundObs )
        {
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
    if( mPhoebe->roverStatus().obstacle().detected )
    {
        mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
        if( mPhoebe->roverStatus().currentState() == NavState::DriveAroundObs )
        {
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
            { NavState::TurnToBall, "Turn to Ball" },
            { NavState::DriveToBall, "Drive to Ball" },
            { NavState::TurnAroundObs, "Turn Around Obstacle"},
            { NavState::DriveAroundObs, "Drive Around Obstacle" },
            { NavState::SearchTurnAroundObs, "Search Turn Around Obstacle" },
            { NavState::SearchDriveAroundObs, "Search Drive Around Obstacle" },
            { NavState::Unknown, "Unknown" }
        };

    return navStateNames.at( mPhoebe->roverStatus().currentState() );
} // stringifyNavState()


// thresholds based on state? waypoint vs ball
// set distance to go around obstacles?
// set threshold for when to skip a point?

// TODOS:
// [drive to ball] obstacle and ball
// all of code, what to do in cases of both ball and obstacle
