#include "stand-client/StandClient.h"
StandClient::StandClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
, LOOP_TIME_LIMIT(5.0)
, logData(false)
, contactReleaseDelay(2.0)
{

}

StandClient::~StandClient()
{

}

bool StandClient::configure(yarp::os::ResourceFinder &rf)
{
    _rf = rf;
    if (rf.check("savePath")) {
        logData = true;
        savePath = rf.find("savePath").asString().c_str();
        savePath = boost::filesystem::canonical(savePath).string();
        std::cout << "savePath: \n" << savePath << std::endl;

    }
    return true;
}

bool StandClient::initialize()
{
    // BUG: The WBI behind the model object is initialized in a separate thread and is not quite ready to be queried as this stage. The delay here is simply to give the WBI enough time to do at least one update. Otherwise, all of the data from the model object is erronius.
    yarp::os::Time::delay(1);


    leftLegContactTask = std::make_shared<ocra_recipes::TaskConnection>("LeftUpperLegContact");
    rightLegContactTask = std::make_shared<ocra_recipes::TaskConnection>("RightUpperLegContact");


    if ( !getComWaypoints() ) {
        return false;
    }

    if(!createDataFiles()) {
        return false;
    }
    if ( logData ) {
        getComBounds();
        getJointLimits();
    }


    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::TIME_OPTIMAL;
    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::STOP_THREAD;

    comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", comWaypointList, trajType, termStrategy);
    comTask = std::make_shared<ocra_recipes::TaskConnection>("ComTask");
    comTask->openControlPorts();

    comTrajThread->setMaxVelocityAndAcceleration(0.1, 0.1);

    contactsReleased = false;

    comTrajThread->start();

    setLoopTimeLimit();
    startTime = yarp::os::Time::now();


    return true;
}

void StandClient::loop()
{
    relativeTime = yarp::os::Time::now() - startTime;
    if (!contactsReleased && (relativeTime >= contactReleaseDelay) ){
        deactivateLegContacts();
        contactsReleased = true;
    }

    if (logData) {
        logClientData();
    }

    if(!comTrajThread->isRunning()) {
        std::cout << "Attained Goal. Stopping." << std::endl;
        stop();
    }

    if (relativeTime > LOOP_TIME_LIMIT) {
        std::cout << "Loop time limit exceeded. Stopping." << std::endl;
        stop();
    }

}

void StandClient::release()
{
    if (logData) {
        writeWaypointsToFile();
        closeDataFiles();
    }
    if(comTrajThread) {
        comTrajThread->stop();
    }
}


bool StandClient::getComWaypoints()
{
    if ( _rf.check("comWptFile") ) {
        comWaypointFilePath = _rf.find("comWptFile").asString().c_str();
        comWaypointFilePath = boost::filesystem::canonical(comWaypointFilePath).string();
        std::cout << "comWaypointFilePath: \n" << comWaypointFilePath << std::endl;
        bool ok = getWaypointDataFromFile(comWaypointFilePath, comWaypointList);

        if (ok) {
            std::cout << "-------------------------------------------------------------------" << std::endl;
            std::cout << "CoM" << std::endl;
            std::cout << "-------------------------------------------------------------------" << std::endl;
            std::cout << "Waypoints: \n";
            for (auto v:comWaypointList){std::cout << v.transpose() << std::endl;}
            std::cout << "\n";
            return true;
        } else {
            return false;
        }

    } else {
        comWaypointList = getManualSolutionWaypoints();
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << "CoM Manual Solution Waypoints" << std::endl;
        std::cout << "-------------------------------------------------------------------" << std::endl;
        for (auto v:comWaypointList){std::cout << v.transpose() << std::endl;}
        std::cout << "\n";
        return true;
    }
}

bool StandClient::createDataFiles()
{
    comPositionRealFilePath = savePath + "/comPositionReal.txt";
    comPositionRefFilePath = savePath + "/comPositionRef.txt";
    torquesFilePath = savePath + "/torques.txt";
    comWaypointsFilePath = savePath + "/comWaypoints.txt";
    timelineFilePath = savePath + "/timeline.txt";
    comExpectedDurationFilePath = savePath + "/comExpectedDuration.txt";
    comBoundsFilePath = savePath + "/comBounds.txt";

    comJacobiansFilePath = savePath + "/comJacobians.txt";
    jointPositionsFilePath = savePath + "/jointPositions.txt";
    jointLimitsFilePath = savePath + "/jointLimits.txt";

    comPositionRealFile.open(comPositionRealFilePath);
    comPositionRefFile.open(comPositionRefFilePath);
    torquesFile.open(torquesFilePath);
    comWaypointsFile.open(comWaypointsFilePath);
    timelineFile.open(timelineFilePath);
    comExpectedDurationFile.open(comExpectedDurationFilePath);
    comBoundsFile.open(comBoundsFilePath);

    comJacobiansFile.open(comJacobiansFilePath);
    jointPositionsFile.open(jointPositionsFilePath);
    jointLimitsFile.open(jointLimitsFilePath);


    bool ok = true;
    ok &= comPositionRealFile.is_open();
    ok &= comPositionRefFile.is_open();
    ok &= torquesFile.is_open();
    ok &= comWaypointsFile.is_open();
    ok &= timelineFile.is_open();
    ok &= comExpectedDurationFile.is_open();
    ok &= comBoundsFile.is_open();
    ok &= comJacobiansFile.is_open();
    ok &= jointPositionsFile.is_open();
    ok &= jointLimitsFile.is_open();
    return ok;
}


void StandClient::closeDataFiles()
{
    comPositionRealFile.close();
    comPositionRefFile.close();
    torquesFile.close();
    comWaypointsFile.close();
    timelineFile.close();
    comExpectedDurationFile.close();
    comBoundsFile.close();
    comJacobiansFile.close();
    jointPositionsFile.close();
    jointLimitsFile.close();
}


bool StandClient::getWaypointDataFromFile(const std::string& filePath, std::list<Eigen::VectorXd>& waypointList)
{
    std::ifstream waypointFile(filePath);
    std::string line;
    if (waypointFile.is_open()) {
        while ( std::getline(waypointFile, line) ) {
            waypointList.push_back(ocra::util::stringToVectorXd(line.c_str()));
        }
        if (waypointList.size()==1) {
            // Add the goal waypoint.
            waypointList.push_back(getComGoalWaypoint());
        }
        waypointFile.close();
        return true;
    } else {
        std::cout << "Unable to open file";
        return false;
    }
}

Eigen::Vector3d StandClient::getComGoalWaypoint()
{
    Eigen::Vector3d comGoalPos = model->getCoMPosition();
    double zDisp = 0.15;
    comGoalPos(0) = 0.0;
    comGoalPos(2) += zDisp;
    return comGoalPos;
}

std::list<Eigen::VectorXd> StandClient::getManualSolutionWaypoints()
{
    Eigen::Vector3d comStartingPos = model->getCoMPosition();

    double zDisp = 0.15;

    /* Manual solution */
    Eigen::MatrixXd com_waypoints(3,2);
    com_waypoints << comStartingPos, comStartingPos;
    com_waypoints(0,0) = 0.0; // First move x forward to between the feet
    com_waypoints(0,1) = 0.0; // First move x forward to between the feet
    com_waypoints(2,1) += zDisp; // and move z upward

    std::list<Eigen::VectorXd> com_waypointList;
    for(int i=0; i<com_waypoints.cols(); ++i) {
        com_waypointList.push_back(com_waypoints.col(i));
    }
    return com_waypointList;
}

void StandClient::getComBounds()
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

    Eigen::Vector3d l_legContact = leftLegContactTask->getTaskState().getPosition().getTranslation();
    Eigen::Vector3d r_legContact = rightLegContactTask->getTaskState().getPosition().getTranslation();


    double x_min = std::fmax(l_legContact(0), r_legContact(0));
    double x_max = std::fmin(r_sole_FrontRight(0), l_sole_FrontLeft(0));

    double y_min = std::fmax(r_sole_BackRight(1), r_sole_FrontRight(1));
    double y_max = std::fmin(l_sole_BackLeft(1), l_sole_FrontLeft(1));

    double z_min = std::fmax(l_legContact(2), r_legContact(2));
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

void StandClient::getJointLimits()
{
    jointLimitsFile << model->getJointLowerLimits().transpose() << "\n";
    jointLimitsFile << model->getJointUpperLimits().transpose() << "\n";
}

void StandClient::logClientData()
{
    timelineFile << relativeTime << "\n";
    comPositionRefFile << comTask->getDesiredTaskState().getPosition().getTranslation().transpose() << "\n";
    comPositionRealFile << model->getCoMPosition().transpose() << "\n";
    torquesFile << model->getJointTorques().transpose() << "\n";

    comJacobian = model->getCoMJacobian();
    comJacobiansFile << Eigen::VectorXd( Eigen::Map<Eigen::VectorXd>(comJacobian.data(), comJacobian.rows()*comJacobian.cols()) ).transpose() << "\n";

    jointPositionsFile << model->getJointPositions().transpose() << "\n";
}

void StandClient::writeWaypointsToFile()
{
    for (auto w : comTrajThread->getWaypointList()) {
        comWaypointsFile << w.transpose() << "\n";
    }
}

void StandClient::setLoopTimeLimit()
{
    double limitFactor = 2.5;
    double comExpectedDuration = comTrajThread->getDuration();
    comExpectedDurationFile << comExpectedDuration;
    LOOP_TIME_LIMIT = comExpectedDuration*limitFactor;
}

void StandClient::deactivateLegContacts()
{
    std::cout << "Deactivating back leg contacts." << std::endl;
    leftLegContactTask->deactivate();
    rightLegContactTask->deactivate();
}