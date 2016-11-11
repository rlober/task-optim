#include "reach-client/ReachClient.h"
ReachClient::ReachClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
, trigger(true)
, LOOP_TIME_LIMIT(5.0)
, logData(false)
, goToHomeOnRelease(false)
, returningHome(false)
, usingComTask(true)
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
    comBoundsFilePath = savePath + "/comBounds.txt";

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
    comBoundsFile.open(comBoundsFilePath);

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
    ok &= comBoundsFile.is_open();
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
    comBoundsFile.close();
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

        if (rf.check("noCom")) {
            usingComTask = false;
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
        comTermStrategy = ocra_recipes::TERMINATION_STRATEGY::NONE;
    } else {
        rightHandTermStrategy = ocra_recipes::TERMINATION_STRATEGY::STOP_THREAD_DEACTIVATE;
        comTermStrategy = ocra_recipes::TERMINATION_STRATEGY::NONE;
    }


    rightHandTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RightHandCartesian", rightHandWaypointList, trajType, rightHandTermStrategy);

    rightHandTrajThread->setGoalErrorThreshold(0.03);


    if (usingComTask) {
        comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", comWaypointList, trajType, comTermStrategy);
    }
    rightHandTask = std::make_shared<ocra_recipes::TaskConnection>("RightHandCartesian");
    comTask = std::make_shared<ocra_recipes::TaskConnection>("ComTask");


    initialRightHandState = rightHandTask->getDesiredTaskState();
    initialComState = comTask->getDesiredTaskState();

    getComBounds();
    return true;
}

void ReachClient::getComBounds()
{
    // <task name="LeftFootContact_BackLeft" type="PointContact">
    // <offset x="-0.02" y="-0.02" z="0.0" qw="0.0" qx="0.707107" qy="0.707107" qz="0.0" />
    //
    // <task name="LeftFootContact_FrontLeft" type="PointContact">
    // <offset x=" 0.06" y="-0.02" z="0.0" qw="0.0" qx="0.707107" qy="0.707107" qz="0.0" />

    Eigen::Vector3d l_sole_center =  model->getSegmentPosition("l_sole").getTranslation();

    Eigen::Vector3d l_sole_FrontLeft = l_sole_center + Eigen::Vector3d(0.06, -0.02, 0.0);
    Eigen::Vector3d l_sole_BackLeft = l_sole_center + Eigen::Vector3d(-0.02, -0.02, 0.0);

    // <task name="RightFootContact_BackRight" type="PointContact">
    // <offset x="-0.02" y=" 0.02" z="0.0" qw="0.0" qx="-0.707107" qy="-0.707107" qz="0.0" />
    //
    // <task name="RightFootContact_FrontRight" type="PointContact">
    // <offset x=" 0.06" y=" 0.02" z="0.0" qw="0.0" qx="-0.707107" qy="-0.707107" qz="0.0" />

    Eigen::Vector3d r_sole_center =  model->getSegmentPosition("r_sole").getTranslation();

    Eigen::Vector3d r_sole_FrontRight = r_sole_center + Eigen::Vector3d(0.06, 0.02, 0.0);
    Eigen::Vector3d r_sole_BackRight = r_sole_center + Eigen::Vector3d(-0.02, 0.02, 0.0);

    double x_min = std::fmax(r_sole_BackRight(0), l_sole_BackLeft(0));
    double x_max = std::fmin(r_sole_FrontRight(0), l_sole_FrontLeft(0));

    double y_min = std::fmax(r_sole_BackRight(1), r_sole_FrontRight(1));
    double y_max = std::fmin(l_sole_BackLeft(1), l_sole_FrontLeft(1));

    double z_min = 0.38; //0.3 --> absolute min, but unstable.
    double z_max = 0.52;

    std::cout << "===================================" << std::endl;
    std::cout << "CoM Bounds:" << std::endl;
    std::cout << x_min << " <= x <= " << x_max << std::endl;
    std::cout << y_min << " <= y <= " << y_max << std::endl;
    std::cout << z_min << " <= z <= " << z_max << std::endl;
    std::cout << "===================================" << std::endl;

    comBoundsFile << x_min << " " << x_max << "\n";
    comBoundsFile << y_min << " " << y_max << "\n";
    comBoundsFile << z_min << " " << z_max;
}

void ReachClient::release()
{

    if (logData) {
        writeWaypointsToFile();
        closeDataFiles();
    }
    if(rightHandTrajThread){rightHandTrajThread->stop();}
    if (usingComTask) {
        if(comTrajThread){comTrajThread->stop();}
    }
    rightHandTask->setDesiredTaskState(initialRightHandState);
    comTask->setDesiredTaskState(initialComState);
}

void ReachClient::loop()
{
    if (trigger) {
        if (usingComTask) {
            comTrajThread->start();
        }
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
            std::cout << "Attained Goal. Returning to home position." << std::endl;
            returningHome = true;
        }
    }

    if (relativeTime > LOOP_TIME_LIMIT) {
        if (!goToHomeOnRelease) {
            std::cout << "Loop time limit exceeded. Stopping." << std::endl;
            stop();
        } else {
            if (!returningHome) {
                std::cout << "Loop time limit exceeded. Returning to home position." << std::endl;
                rightHandTrajThread->returnToHome();
                if (usingComTask) {
                    comTrajThread->returnToHome();
                }
                returningHome = true;
            }
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
    if (usingComTask) {
        for (auto w : comTrajThread->getWaypointList()) {
            comWaypointsFile << w.transpose() << "\n";
        }
    }
    for (auto w : rightHandTrajThread->getWaypointList()) {
        rightHandWaypointsFile << w.transpose() << "\n";
    }
}

void ReachClient::setLoopTimeLimit()
{
    double rightHandExpectedDuration = rightHandTrajThread->getDuration();
    rightHandExpectedDurationFile << rightHandExpectedDuration;

    if (usingComTask) {
        double comExpectedDuration = comTrajThread->getDuration();
        comExpectedDurationFile << comExpectedDuration;
        // Make time limit 3x the longest traj.
        LOOP_TIME_LIMIT = std::fmax(comExpectedDuration, rightHandExpectedDuration)*3.0;
    } else {
        LOOP_TIME_LIMIT = rightHandExpectedDuration*3.0;
    }
}
