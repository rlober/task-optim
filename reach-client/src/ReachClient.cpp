#include "reach-client/ReachClient.h"
ReachClient::ReachClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
, trigger(true)
, LOOP_TIME_LIMIT(5.0)
, logData(false)
, goToHomeOnRelease(false)
, returningHome(false)
{

}

ReachClient::~ReachClient()
{

}

bool ReachClient::createDataFiles()
{
    rightHandPositionRealFilePath = savePath + "/rightHandPositionReal.txt";
    rightHandPositionRefFilePath = savePath + "/rightHandPositionRef.txt";
    comPositionRealFilePath = savePath + "/comPositionReal.txt";
    comPositionRefFilePath = savePath + "/comPositionRef.txt";
    torquesFilePath = savePath + "/torques.txt";
    comWaypointsFilePath = savePath + "/comWaypoints.txt";
    rightHandWaypointsFilePath = savePath + "/rightHandWaypoints.txt";
    timelineFilePath = savePath + "/timeline.txt";
    comExpectedDurationFilePath = savePath + "/comExpectedDuration.txt";
    rightHandExpectedDurationFilePath = savePath + "/rightHandExpectedDuration.txt";

    rightHandPositionRealFile.open(rightHandPositionRealFilePath);
    rightHandPositionRefFile.open(rightHandPositionRefFilePath);
    comPositionRealFile.open(comPositionRealFilePath);
    comPositionRefFile.open(comPositionRefFilePath);
    torquesFile.open(torquesFilePath);
    comWaypointsFile.open(comWaypointsFilePath);
    rightHandWaypointsFile.open(rightHandWaypointsFilePath);
    timelineFile.open(timelineFilePath);
    comExpectedDurationFile.open(comExpectedDurationFilePath);
    rightHandExpectedDurationFile.open(rightHandExpectedDurationFilePath);

    bool ok = true;
    ok &= rightHandPositionRealFile.is_open();
    ok &= rightHandPositionRefFile.is_open();
    ok &= comPositionRealFile.is_open();
    ok &= comPositionRefFile.is_open();
    ok &= torquesFile.is_open();
    ok &= comWaypointsFile.is_open();
    ok &= rightHandWaypointsFile.is_open();
    ok &= timelineFile.is_open();
    ok &= comExpectedDurationFile.is_open();
    ok &= rightHandExpectedDurationFile.is_open();
    return ok;
}


void ReachClient::closeDataFiles()
{
    rightHandPositionRealFile.close();
    rightHandPositionRefFile.close();
    comPositionRealFile.close();
    comPositionRefFile.close();
    torquesFile.close();
    comWaypointsFile.close();
    rightHandWaypointsFile.close();
    timelineFile.close();
    comExpectedDurationFile.close();
    rightHandExpectedDurationFile.close();
}

bool ReachClient::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("rightHandWptFile") && rf.check("comWptFile") ) {
        rightHandWaypointFilePath = rf.find("rightHandWptFile").asString().c_str();
        comWaypointFilePath = rf.find("comWptFile").asString().c_str();
        rightHandWaypointFilePath = boost::filesystem::canonical(rightHandWaypointFilePath).string();
        comWaypointFilePath = boost::filesystem::canonical(comWaypointFilePath).string();

        std::cout << "rightHandWaypointFilePath: \n" << rightHandWaypointFilePath << std::endl;
        std::cout << "comWaypointFilePath: \n" << comWaypointFilePath << std::endl;
        bool ok = getWaypointDataFromFile(rightHandWaypointFilePath, rightHandWaypointList);
        ok &= getWaypointDataFromFile(comWaypointFilePath, comWaypointList);
        if (rf.check("savePath")) {
            logData = true;
            savePath = rf.find("savePath").asString().c_str();
            savePath = boost::filesystem::canonical(savePath).string();
            std::cout << "savePath: \n" << savePath << std::endl;
            ok &= createDataFiles();
        }

        if (rf.check("home")) {
            goToHomeOnRelease = true;
        }
        if (ok) {
            rightHandGoalPosition = *rightHandWaypointList.rbegin();
            comGoalPosition = *comWaypointList.rbegin();
            std::cout << "-------------------------------------------------------------------" << std::endl;
            std::cout << "Right Hand" << std::endl;
            std::cout << "-------------------------------------------------------------------" << std::endl;
            std::cout << "Starting position: " <<  model->getSegmentPosition("r_hand").getTranslation().transpose()  << std::endl;
            std::cout << "Goal position: " <<  rightHandGoalPosition.transpose()  << std::endl;
            std::cout << "Waypoints: \n";
            for (auto v:rightHandWaypointList){std::cout << v.transpose() << std::endl;}
            std::cout << "\n";
            std::cout << "-------------------------------------------------------------------" << std::endl;
            std::cout << "CoM" << std::endl;
            std::cout << "-------------------------------------------------------------------" << std::endl;
            std::cout << "Starting position: " <<  model->getCoMPosition().transpose()  << std::endl;
            std::cout << "Goal position: " <<  comGoalPosition.transpose()  << std::endl;
            std::cout << "Waypoints: \n";
            for (auto v:comWaypointList){std::cout << v.transpose() << std::endl;}
            std::cout << "\n";
            return true;
        } else {
            return false;
        }

    } else {
        return false;
    }
}

bool ReachClient::getWaypointDataFromFile(const std::string& filePath, std::list<Eigen::VectorXd>& waypointList)
{
    std::ifstream waypointFile(filePath);
    std::string line;
    if (waypointFile.is_open()) {
        while ( std::getline(waypointFile, line) ) {
            waypointList.push_back(ocra::util::stringToVectorXd(line.c_str()));
        }
        waypointFile.close();
        return true;
    } else {
        std::cout << "Unable to open file";
        return false;
    }
}

bool ReachClient::initialize()
{
    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::TIME_OPTIMAL;

    if (goToHomeOnRelease) {
        rightHandTermStrategy = ocra_recipes::TERMINATION_STRATEGY::REVERSE_STOP;
        comTermStrategy = ocra_recipes::TERMINATION_STRATEGY::REVERSE_STOP;
    } else {
        rightHandTermStrategy = ocra_recipes::TERMINATION_STRATEGY::STOP_THREAD_DEACTIVATE;
        comTermStrategy = ocra_recipes::TERMINATION_STRATEGY::WAIT;
    }


    rightHandTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RightHandCartesian", rightHandWaypointList, trajType, rightHandTermStrategy);

    rightHandTrajThread->setGoalErrorThreshold(0.03);

    comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", comWaypointList, trajType, comTermStrategy);

    rightHandTask = std::make_shared<ocra_recipes::TaskConnection>("RightHandCartesian");
    comTask = std::make_shared<ocra_recipes::TaskConnection>("ComTask");

    initialRightHandState = rightHandTask->getDesiredTaskState();
    initialComState = comTask->getDesiredTaskState();
    return true;
}

void ReachClient::release()
{

    if (logData) {
        writeWaypointsToFile();
        closeDataFiles();
    }
    if(rightHandTrajThread){rightHandTrajThread->stop();}
    if(comTrajThread){comTrajThread->stop();}
    rightHandTask->setDesiredTaskState(initialRightHandState);
    comTask->setDesiredTaskState(initialComState);
}

void ReachClient::loop()
{
    if (trigger) {
        comTrajThread->start();
        rightHandTrajThread->start();
        startTime = yarp::os::Time::now();
        setLoopTimeLimit();
        trigger = false;
    }
    relativeTime = yarp::os::Time::now() - startTime;

    if (rightHandTrajThread->goalAttained()) {
        if (!goToHomeOnRelease) {
            std::cout << "Attained Goal. Stopping." << std::endl;
            stop();
        } else if (returningHome) {
            stop();
        } else {
            returningHome = true;
        }
    }

    if (relativeTime > LOOP_TIME_LIMIT) {
        if (!goToHomeOnRelease) {
            std::cout << "Loop time limit exceeded. Stopping." << std::endl;
            stop();
        }
    }

    if (logData) {
        logClientData();
    }

}

void ReachClient::logClientData()
{
    timelineFile << relativeTime << "\n";
    rightHandPositionRefFile << rightHandTask->getDesiredTaskState().getPosition().getTranslation().transpose() << "\n";
    rightHandPositionRealFile << model->getSegmentPosition("r_hand").getTranslation().transpose() << "\n";
    comPositionRefFile << comTask->getDesiredTaskState().getPosition().getTranslation().transpose() << "\n";
    comPositionRealFile << model->getCoMPosition().transpose() << "\n";
    torquesFile << model->getJointTorques().transpose() << "\n";
}

void ReachClient::writeWaypointsToFile()
{
    for (auto w : comTrajThread->getWaypointList()) {
        comWaypointsFile << w.transpose() << "\n";
    }
    for (auto w : rightHandTrajThread->getWaypointList()) {
        rightHandWaypointsFile << w.transpose() << "\n";
    }
}

void ReachClient::setLoopTimeLimit()
{
    double comExpectedDuration = comTrajThread->getDuration();
    double rightHandExpectedDuration = rightHandTrajThread->getDuration();
    comExpectedDurationFile << comExpectedDuration;
    rightHandExpectedDurationFile << rightHandExpectedDuration;

    // Make time limit 3x the longest traj.
    LOOP_TIME_LIMIT = std::fmax(comExpectedDuration, rightHandExpectedDuration)*3.0;
}
