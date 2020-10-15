#include auton_armStateMachine.cpp
//***include other stuff


// Constructs a StateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine( lcm::LCM& lcmObject )
    : mPhoebe( nullptr )
    , mLcmObject( lcmObject )
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

// Updates the target information of the rover's status.
void StateMachine::updateRoverStatus( TargetList targetList )
{
    Target target1 = targetList.targetList[0];
    Target target2 = targetList.targetList[1];
    mNewRoverStatus.target() = target1;
    mNewRoverStatus.target2() = target2;
} // updateRoverStatus( Target )

