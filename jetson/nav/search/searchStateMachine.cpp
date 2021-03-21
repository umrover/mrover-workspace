#include "searchStateMachine.hpp"

#include "stateMachine.hpp"
#include "utilities.hpp"
#include "spiralOutSearch.hpp"
#include "spiralInSearch.hpp"
#include "lawnMowerSearch.hpp"

#include <iostream>
#include <time.h>
#include <cmath>

// Constructs an SearchStateMachine object with roverStateMachine
SearchStateMachine::SearchStateMachine( StateMachine* roverStateMachine )
    : roverStateMachine( roverStateMachine ) {}

// Runs the search state machine through one iteration. This will be called by
// StateMachine  when NavState is in a search state. This will call the corresponding
NavState SearchStateMachine::run( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    switch ( phoebe->roverStatus().currentState() )
    {
        case NavState::SearchSpin:
        {
            return executeSearchSpin( phoebe, roverConfig );
        }

        case NavState::SearchGimbal:
        {
            return executeSearchGimbal( phoebe, roverConfig );
        }
        case NavState::SearchWait:
        case NavState::TurnedToTargetWait:
        {
            return executeRoverWait( phoebe, roverConfig );
        }

        case NavState::SearchTurn:
        {
            return executeSearchTurn( phoebe, roverConfig );
        }

        case NavState::SearchDrive:
        {
            return executeSearchDrive( phoebe, roverConfig );
        }

        case NavState::TurnToTarget:
        {
            return executeTurnToTarget( phoebe, roverConfig );
        }

        case NavState::DriveToTarget:
        {
            return executeDriveToTarget( phoebe, roverConfig );
        }

        default:
        {
            cerr << "Entered Unknown NavState in search state machine" << endl;
            return NavState::Unknown;
        }
    } // switch
} // run()

// Executes the logic for a search spin. If at a multiple of
// waitStepSize, the rover will go to SearchWait. If the rover
// detects the target, it proceeds to the target. If finished with a 360,
// the rover moves on to the next phase of the search. Else continues
// to search spin.
NavState SearchStateMachine::executeSearchSpin( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    // degrees to turn to before performing a search wait.
    double waitStepSize = roverConfig[ "search" ][ "searchWaitStepSize" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; // initialize, is corrected on first call

    if( phoebe->roverStatus().leftTarget().distance >= 0 )
    {
        updateTargetDetectionElements( phoebe->roverStatus().leftTarget().bearing,
                                           phoebe->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    if( nextStop == 0 )
    {
        // get current angle and set as origAngle
        mOriginalSpinAngle = phoebe->roverStatus().odometry().bearing_deg; // doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if( phoebe->turn( nextStop ) )
    {
        if( nextStop - mOriginalSpinAngle >= 360 )
        {
            nextStop = 0;
            return NavState::SearchTurn;
        }
        nextStop += waitStepSize;
        return NavState::SearchWait;
    }
    return NavState::SearchSpin;
} // executeSearchSpin()


//Executes the logic for a gimbal search spin. The main objective of a gimbal search spin is to spin the gimbal
//to positive "gimbalSearchAngleMag" (150) then to -150 then to 0. Every "wait step size" we stop the gimbal
//in order to give it time to find the target. Only use the wait step size on the 1st phase of the rotation
//this is because its unnecsesary to double check
NavState SearchStateMachine::executeSearchGimbal( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    //initially set the waitstepsize to be the same as the gimbalSearchAngleMag so we just go straight to
    //the extremity without waiting.
    static double waitStepSize = roverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double phase = 0; // if 0, go to +150. if 1 go to -150, if 2 go to 0
    static double desiredYaw = roverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble(); 

    //if target aquired, turn to it
    if( phoebe->roverStatus().leftTarget().distance >= 0 )
    {
        cout  << "Target Aquired! gimbal" << endl;
        updateTargetDetectionElements( phoebe->roverStatus().leftTarget().bearing,
                                           phoebe->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    //set the desiredYaw to wherever the next stop on the gimbals path is
    //enter the if if the gimbal is at the next stop
    if( phoebe->gimbal().setDesiredGimbalYaw( nextStop ) )
    {   
        //if the next stop is at the desiredYaw for the phase (150, -150, 0)
        if ( nextStop == desiredYaw )
        {
            //if there are more phases, increment the phase
            if ( phase <= 2 )
                ++phase;
            //if the phase is one, set the waitstepsize to the specified config value and flip desired yaw
            //goal of this phase is to go in waitstepsize increments from positive gimbalSearchAngleMag to 
            //negative gimbalSearchAngleMag
            if ( phase == 1 ) {
                waitStepSize = -roverConfig[ "search" ][ "gimbalSearchWaitStepSize" ].GetDouble();
                desiredYaw *= -1;
            }
            //Go straight to zero, set the waitstep size to the difference between 0 and currentPosition
            else if ( phase == 2 ) 
            {
                waitStepSize = 0 - nextStop;
                desiredYaw = 0;
            }
        }
       
        //if we are done with all phases
        if ( phase == 3 )
        {
            //reset static vars
            waitStepSize = roverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble();
            nextStop = 0;
            phase = 0;
            desiredYaw = roverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble( );
            //Turn to next search point
            return NavState::SearchTurn;
        }
        //set the next stop for the gimbal to increment by the waitStepSize
        nextStop += waitStepSize;
        //we are at our stopping point for the camera so go into search gimbal wait
        return NavState::SearchWait;
    }
    //publish gimbal lcm command
    phoebe->publishGimbal( );

    return NavState::SearchGimbal;
}


// Executes the logic for waiting during a search spin so that CV can
// look for the target. If the rover detects the target, it proceeds
// to the target. If the rover is done waiting, it continues the search
// spin. Else the rover keeps waiting.
NavState SearchStateMachine::executeRoverWait( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    static bool started = false;
    static time_t startTime;

    if( phoebe->roverStatus().leftTarget().distance >= 0 )
    {
        updateTargetDetectionElements( phoebe->roverStatus().leftTarget().bearing,
                                       phoebe->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    if( !started )
    {
        phoebe->stop();
        startTime = time( nullptr );
        started = true;
    }
    double waitTime = roverConfig[ "search" ][ "searchWaitTime" ].GetDouble();
    if( difftime( time( nullptr ), startTime ) > waitTime )
    {
        started = false;
        // Determine wwhich Spin we are executing then go back to the correct method
        if (roverConfig[ "search" ][ "useGimbal" ].GetBool())
        {
            return NavState::SearchGimbal;
        }
        else {
            return NavState::SearchSpin;
        }
        return NavState::SearchTurn;
    }
    else
    {
        if ( phoebe->roverStatus().currentState() == NavState::SearchWait )
        {
            return NavState::SearchWait;
        }
        return NavState::TurnedToTargetWait;
    }
}

// Executes the logic for turning while searching.
// If no remaining search points, it proceeds to change search algorithms.
// If the rover detects the target, it proceeds to the target.
// If the rover finishes turning, it proceeds to driving while searching.
// Else the rover keeps turning to the next Waypoint.
NavState SearchStateMachine::executeSearchTurn( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    cout << "search size: " << mSearchPoints.size() << endl;
    if( mSearchPoints.empty() )
    {
        cout << "changing search alg" << endl;
        return NavState::ChangeSearchAlg;
    }
    if( phoebe->roverStatus().leftTarget().distance >= 0 )
    {
        updateTargetDetectionElements( phoebe->roverStatus().leftTarget().bearing,
                                       phoebe->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    Odometry& nextSearchPoint = mSearchPoints.front();
    if( phoebe->turn( nextSearchPoint ) )
    {
        return NavState::SearchDrive;
    }
    return NavState::SearchTurn;
} // executeSearchTurn()

// Executes the logic for driving while searching.
// If the rover detects the target, it proceeds to the target.
// If the rover detects an obstacle, it proceeds to obstacle avoidance.
// If the rover finishes driving, it proceeds to turning to the next Waypoint.
// If the rover is still on course, it keeps driving to the next Waypoint.
// Else the rover turns to the next Waypoint or turns back to the current Waypoint
NavState SearchStateMachine::executeSearchDrive( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    if( phoebe->roverStatus().leftTarget().distance >= 0 )
    {
        updateTargetDetectionElements( phoebe->roverStatus().leftTarget().bearing,
                                           phoebe->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    if( isObstacleDetected( phoebe ) )
    {
        roverStateMachine->updateObstacleAngle( phoebe->roverStatus().obstacle().bearing );
        roverStateMachine->updateObstacleDistance( phoebe->roverStatus().obstacle().distance );
        return NavState::SearchTurnAroundObs;
    }
    const Odometry& nextSearchPoint = mSearchPoints.front();
    DriveStatus driveStatus = phoebe->drive( nextSearchPoint );

    if( driveStatus == DriveStatus::Arrived )
    {
        mSearchPoints.pop_front();
    
        //if the rover uses the gimbal use it, otherwise dont.
        if ( roverConfig[ "search" ][ "useGimbal" ].GetBool() ){
            return NavState::SearchGimbal;
        }
        else{
            return NavState::SearchSpin;
        }
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::SearchDrive;
    }
    return NavState::SearchTurn;
} // executeSearchDrive()

// Executes the logic for turning to the target.
// Starts by turnign the gimbal towards the target. Then holds for 200 cycles to confirm aquisition
// If the rover loses the target, will continue to turn using last known angles.
// Once target location confirmed, rover turns towards the target, then re-centers its gimbal
// If the rover finishes turning to the target, it goes into waiting state to
// give CV time to relocate the target
// Else the rover continues to turn to to the target.
NavState SearchStateMachine::executeTurnToTarget( Rover* phoebe, const rapidjson::Document& roverConfig  )
{
    static int turnToTargetStage = 0;
    static double bearingAngle = 0;

    static bool firstTime = true;
    
    //if we are not using a gimbal, just skip all gimbal states and immediately turn to target, only needs to run once
    if ( !roverConfig[ "search" ][ "useGimbal" ].GetBool() && firstTime){
        turnToTargetStage = 5;
        firstTime = false;
        bearingAngle = phoebe->roverStatus().leftTarget().bearing + phoebe->roverStatus().odometry().bearing_deg;
    }
    
    //if we lose the target and we are not in a state where this is expected (re-centering gimbal, turning to target (gimbal might go off))
    //then notify loss of target, reset static vars, and turn back to last known angle on the search path
    if( phoebe->roverStatus().leftTarget().distance < 0  && turnToTargetStage != 2 && turnToTargetStage != 3)
    {
        cerr << "Lost the target. Continuing to turn to last known angle\n";
        if( phoebe->turn( mTargetAngle + mTurnToTargetRoverAngle ) )
        {
            turnToTargetStage = 0;
            bearingAngle = 0;
            firstTime = true;
            return NavState::TurnedToTargetWait;
        }
        return NavState::TurnToTarget;
    }
   
    
    //first step (with gimbal): turn gimbal to the target with a margin of error of 10 degrees
    if ( turnToTargetStage == 0 ){
       
        if ( abs( phoebe->roverStatus().leftTarget().bearing - phoebe->gimbal().getYaw() ) > 10 )
        {
            phoebe->gimbal( ).setDesiredGimbalYaw( phoebe->roverStatus().leftTarget().bearing);
        }
        else{
            bearingAngle = phoebe->roverStatus( ).leftTarget( ).bearing + phoebe->roverStatus( ).odometry( ).bearing_deg;
            ++turnToTargetStage;
        }
    }
    //wait "searchWaitTime" seconds to confirm target aquisition, if we don't have a gimbal send to finishing state otherwise just advance
    else if ( turnToTargetStage == 1 ){
        static bool timeStarted = false;
        static int startTime = time( nullptr );
        if (!timeStarted){
            startTime = time( nullptr ); 
            
            timeStarted = true;
        }
        double waitTime = roverConfig[ "search" ][ "searchWaitTime" ].GetDouble();
        if ( difftime( time( nullptr ), startTime ) > waitTime ){
            timeStarted = false;
            if ( !roverConfig["search"]["useGimbal"].GetBool() )
            {
                turnToTargetStage = 4;
            }
            ++turnToTargetStage;
        }
    }
    //turn the rover to face the target (its ok to lose the target here since the gimbal will face away for some time)
    else if ( turnToTargetStage == 2 ){
        
        if( phoebe->turn( bearingAngle ) )
        {   
            ++turnToTargetStage;
        }
    }
    //set the gimbal back to 0 degrees so it is facing the direction of rover travel (it is also ok to lose the target here since it may start facing away from target)
    else if ( turnToTargetStage == 3 ){
        if ( phoebe->gimbal().setDesiredGimbalYaw( 0 ) )
        {
            ++turnToTargetStage;
        }
    }
    //reset static vars and change state to drive towards target
    else if ( turnToTargetStage == 4 ){
        turnToTargetStage = 0;
        bearingAngle = 0;
        firstTime = true;
        return NavState::DriveToTarget;
    }
    //if we don't have a gimbal, simply turn the rover to target and then switch to waiting state to confirm
    else if ( turnToTargetStage == 5 ){
        if( phoebe->turn( bearingAngle ) )
        {   
            turnToTargetStage = 1;
        }
    }

    updateTargetDetectionElements( phoebe->roverStatus().leftTarget().bearing,
                                       phoebe->roverStatus().odometry().bearing_deg );
    //publish gimbal lcm command
    phoebe->publishGimbal( );
    return NavState::TurnToTarget;
} // executeTurnToTarget()

// Executes the logic for driving to the target.
// If the rover loses the target, it continues with search by going to
// the last point before the rover turned to the target
// If the rover detects an obstacle, it proceeds to go around the obstacle.
// If the rover finishes driving to the target, it moves on to the next Waypoint.
// If the rover is on course, it keeps driving to the target.
// Else, it turns back to face the target.
NavState SearchStateMachine::executeDriveToTarget( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    if( phoebe->roverStatus().leftTarget().distance < 0 )
    {
        cerr << "Lost the target\n";
        return NavState::SearchTurn; // NavState::SearchSpin
    }
    if( isObstacleDetected( phoebe ) &&
        !isTargetReachable( phoebe, roverConfig ) )
    {
        roverStateMachine->updateObstacleAngle( phoebe->roverStatus().obstacle().bearing );
        roverStateMachine->updateObstacleDistance( phoebe->roverStatus().obstacle().distance );
        return NavState::SearchTurnAroundObs;
    }
    
    DriveStatus driveStatus;

    double distance = phoebe->roverStatus().leftTarget().distance;
    double bearing = phoebe->roverStatus().leftTarget().bearing + phoebe->roverStatus().odometry().bearing_deg;

    // Executes the logic for driving with 0, 1, or 2 targets in sight
    // If we have a second target detected, determine which post is closer
    // If the distance to the second target is less than the first,
    // set our variables to the target 2's distance and bearing
    // Else, use the initialized values from target 1 when driving
    if( phoebe->roverStatus().rightTarget().distance > 0 )
    {
        if( phoebe->roverStatus().leftTarget().distance > phoebe->roverStatus().rightTarget().distance ) 
        {
            distance = phoebe->roverStatus().rightTarget().distance;
            bearing = phoebe->roverStatus().rightTarget().bearing + phoebe->roverStatus().odometry().bearing_deg;
        }
    }

    driveStatus = phoebe->drive( distance, bearing, true );
    
    if( driveStatus == DriveStatus::Arrived )
    {
        mSearchPoints.clear();
        if( phoebe->roverStatus().path().front().gate )
        {
            roverStateMachine->mGateStateMachine->mGateSearchPoints.clear();
            const double absAngle = mod( phoebe->roverStatus().odometry().bearing_deg +
                                        phoebe->roverStatus().leftTarget().bearing,
                                        360 );
            roverStateMachine->mGateStateMachine->lastKnownRightPost.odom = createOdom( phoebe->roverStatus().odometry(),
                                                                                    absAngle,
                                                                                    phoebe->roverStatus().leftTarget().distance,
                                                                                    phoebe );
            roverStateMachine->mGateStateMachine->lastKnownRightPost.id = phoebe->roverStatus().leftTarget().id;
            if (roverConfig["search"]["useGimbal"].GetBool()) {
                cout << "going into gate search gimbal. Last known right post ID: " <<  roverStateMachine->mGateStateMachine->lastKnownRightPost.id << endl;
                return NavState::GateSearchGimbal;
            }
            else {
                 return NavState::GateSpin;
            }
           
        }
        phoebe->roverStatus().path().pop_front();
        roverStateMachine->updateCompletedPoints();
        return NavState::Turn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::DriveToTarget;
    }
    return NavState::TurnToTarget;
} // executeDriveToTarget()

// Sets last known target angle, so if target is lost, we
// continue to turn to that angle
void SearchStateMachine::updateTargetAngle( double bearing )
{
    mTargetAngle = bearing;
} // updateTargetAngle

// Sets last known rover angle while it is turning to the target
// in case target is lost while turning
void SearchStateMachine::updateTurnToTargetRoverAngle( double bearing )
{
    mTurnToTargetRoverAngle = bearing;
} // updateTurnToTargetRoverAngle

// Sets last known angles for both the target and the rover
// these bearings are used to turn towards the target in case target
// is lost while turning
void SearchStateMachine::updateTargetDetectionElements( double target_bearing, double rover_bearing )
{
    updateTargetAngle( target_bearing );
    updateTurnToTargetRoverAngle( rover_bearing );
} // updateTargetDetectionElements

// add intermediate points between the existing search points in a path generated by a search algorithm.
// The maximum separation between any points in the search point list is determined by the rover's sight distance.
void SearchStateMachine::insertIntermediatePoints( Rover * phoebe, const rapidjson::Document& roverConfig )
{
    double visionDistance = roverConfig[ "computerVision" ][ "visionDistance" ].GetDouble();
    const double maxDifference = 2 * visionDistance;

    for( int i = 0; i < int( mSearchPoints.size() ) - 1; ++i )
    {
        Odometry point1 = mSearchPoints.at( i );
        Odometry point2 = mSearchPoints.at( i + 1 );
        double distance = estimateNoneuclid( point1, point2 );
        if ( distance > maxDifference )
        {
            int numPoints = int( ceil( distance / maxDifference ) - 1 );
            double newDifference = distance / ( numPoints + 1 );
            double bearing = calcBearing( point1, point2 );
            for ( int j = 0; j < numPoints; ++j )
            {
                Odometry startPoint = mSearchPoints.at( i );
                Odometry newOdom = createOdom( startPoint, bearing, newDifference, phoebe );
                auto insertPosition = mSearchPoints.begin() + i + 1;
                mSearchPoints.insert( insertPosition, newOdom );
                ++i;
            }
        }
    }
} // insertIntermediatePoints()

// The search factory allows for the creation of search objects and
// an ease of transition between search algorithms
SearchStateMachine* SearchFactory( StateMachine* stateMachine, SearchType type )  //TODO
{
    SearchStateMachine* search = nullptr;
    switch ( type )
    {
        case SearchType::SPIRALOUT:
            search = new SpiralOut( stateMachine );
            break;

        case SearchType::LAWNMOWER:
            search = new LawnMower( stateMachine );
            break;

        case SearchType::SPIRALIN:
            search = new SpiralIn( stateMachine );
            break;

        default:
            std::cerr << "Unkown Search Type. Defaulting to Spiral\n";
            search = new SpiralOut( stateMachine );
            break;
    }
    return search;
} // SearchFactory

/******************/
/* TODOS */
/******************/
// save location of target then go around object? ( Execute drive to target )
// look into removing the waiting when turning to target or at least doing this a better way. This should at very least be it's own state
