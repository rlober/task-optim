#include "reach-client/ReachClient.h"
ReachClient::ReachClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
, trigger(true)
, LOOP_TIME_LIMIT(15.0)
{

}

ReachClient::~ReachClient()
{

}

bool ReachClient::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("rightHandWptFile") && rf.check("comWptFile") ) {
        rightHandWaypointFilePath = rf.find("rightHandWptFile").asString().c_str();
        comWaypointFilePath = rf.find("comWptFile").asString().c_str();

        bool ok = getWaypointDataFromFile(rightHandWaypointFilePath);
        ok &= getWaypointDataFromFile(comWaypointFilePath);

        return ok;

    } else {
        return false;
    }
}

bool ReachClient::getWaypointDataFromFile(const std::string& filePath)
{

    waypoint.resize(3);
    waypoint << 0.25, -0.2, 0.6;
    rightHandWaypointList.push_back(waypoint);
    waypoint << 0.25, 0.0, 0.6;
    rightHandWaypointList.push_back(waypoint);
    waypoint << 0.25, 0.0, 0.7;
    rightHandWaypointList.push_back(waypoint);
    waypoint << 0.25, -0.2, 0.7;
    rightHandWaypointList.push_back(waypoint);

    rightHandGoalPosition = *rightHandWaypointList.rbegin();
    // comGoalPosition = *comWaypointList.rbegin()


    return true;
}

bool ReachClient::initialize()
{

    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::TIME_OPTIMAL;
    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::CYCLE;
    rightHandTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RightHandCartesian", rightHandWaypointList, trajType, termStrategy);
    rightHandTrajThread->setGoalErrorThreshold(0.03);



    // termStrategy = ocra_recipes::WAIT;
    // comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", comWaypointList, trajType, termStrategy);
    // // comTrajThread->start();
    rightHandTrajThread->start();
    startTime = yarp::os::Time::now();
    return true;
}

void ReachClient::release()
{
    if(rightHandTrajThread){rightHandTrajThread->stop();}
}

void ReachClient::loop()
{
    relativeTime = yarp::os::Time::now() - startTime;

    // if (!rightHandTrajThread->goalAttained()) {
    //     std::cout << "Attained Goal. Stopping." << std::endl;
    //     stop();
    // }

    // if (relativeTime > LOOP_TIME_LIMIT) {
    //     std::cout << "Loop time limit exceeded. Stopping." << std::endl;
    //     this->stop();
    // }

    logClientData();
    // std::cout << "rightHandPosition: " << rightHandPosition.transpose() << std::endl;

}

void ReachClient::logClientData()
{
    rightHandPosition = model->getSegmentPosition(model->getSegmentIndex("r_hand")).getTranslation();
    comPosition = model->getCoMPosition();
    torques = model->getJointTorques();

}
