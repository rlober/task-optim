#include "stand-client/StandClient.h"
StandClient::StandClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
, LOOP_TIME_LIMIT(5.0)
, logData(false)
, contactReleaseDelay(2.0)
, connectedToForcePort(false)
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


        std::string l_foot_FT_port_name("/StandClient/l_foot_FT_port:i");
        std::string r_foot_FT_port_name("/StandClient/r_foot_FT_port:i");
        std::string chair_FT_port_name("/StandClient/chair_FT_port:i");

        l_foot_FT_port.open(l_foot_FT_port_name);
        r_foot_FT_port.open(r_foot_FT_port_name);
        chair_FT_port.open(chair_FT_port_name);

        if( !yarp.connect("/icubSim/left_foot/analog:o", l_foot_FT_port_name) ) {
            std::cout << "[ERROR] Could not connect to l_foot_FT_port!" << std::endl;
            return false;
        }
        if( !yarp.connect("/icubSim/right_foot/analog:o", r_foot_FT_port_name) ) {
            std::cout << "[ERROR] Could not connect to r_foot_FT_port!" << std::endl;
            return false;
        }
        if( !yarp.connect("/chair/FT_sensor/analog:o/forceTorque", chair_FT_port_name) ) {
            std::cout << "[ERROR] Could not connect to chair_FT_port!" << std::endl;
            return false;
        }
    }

    if (rf.check("record")) {
        recordSimulation = true;
        if (rf.check("recordDir")) {
            recordDir = rf.find("recordDir").asString().c_str();
            recordDir = boost::filesystem::canonical(recordDir).string();
        } else {
            recordDir = savePath;
        }

        if (rf.check("recordName")) {
            recordName = rf.find("recordName").asString().c_str();
        } else {
            recordName = "standing";
        }

        if (rf.check("recordDelay")) {
            recordDelay = rf.find("recordDelay").asDouble();
        } else {
            recordDelay = 0.0;
        }

        std::cout << "recordDir: " << recordDir << std::endl;
        std::cout << "recordName: " << recordName << std::endl;
        std::cout << "recordDelay: " << recordDelay << std::endl;

        std::string cameraPortName("/StandClient/camera/rpc:o");
        cameraPort.open(cameraPortName);
        yarp.connect(cameraPortName, "/Gazebo/yarp_camera_sensor/rpc:i");

        // std::string contactPortName("/StandClient/contact:o");
        // contactPort.open(contactPortName);
        // yarp.connect(contactPortName, "/Gazebo/SeatContactPlugin/l_foot_contact_sensor:i");
        // yarp.connect(contactPortName, "/Gazebo/SeatContactPlugin/r_foot_contact_sensor:i");
        // yarp.connect(contactPortName, "/Gazebo/SeatContactPlugin/r_upper_leg_contact_sensor:i");
        // yarp.connect(contactPortName, "/Gazebo/SeatContactPlugin/l_upper_leg_contact_sensor:i");
        // yarp.connect(contactPortName, "/Gazebo/SeatContactPlugin/ground_contact_sensor:i");

    } else {
        recordSimulation = false;
    }

    if (rf.check("applyForce")) {
        std::string gazeboApplyForcePortName("/ReachClient/gazebo/rpc:o");
        gazeboForcePort.open(gazeboApplyForcePortName);
        connectedToForcePort = yarp.connect(gazeboApplyForcePortName, "/Gazebo/ApplyWristForce:i");
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
        std::cout << "[ERROR] Couldn't read CoM waypoints." << std::endl;
        return false;
    }

    if ( logData ) {
        if(!createDataFiles()) {
            return false;
        }
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

    if (recordSimulation) {
        startRecording();
    }

    if ( connectedToForcePort ) {
        yarp::os::Bottle b;
        b.addDouble(3.7);
        b.addDouble(0.0);
        b.addDouble(3.7);
        gazeboForcePort.write(b);
    }

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
        l_foot_FT_port.close();
        r_foot_FT_port.close();
        chair_FT_port.close();
    }

    if (recordSimulation) {
        stopRecording();
        cameraPort.close();
        // contactPort.close();
    }

    if(comTrajThread) {
        comTrajThread->stop();
    }

    if (connectedToForcePort) {
        gazeboForcePort.close();
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
    contactLocationsFilePath = savePath + "/contactLocations.txt";

    l_footForceTorqueFilePath = savePath + "/l_footForceTorques.txt";
    r_footForceTorqueFilePath = savePath + "/r_footForceTorques.txt";
    chairForceTorqueFilePath = savePath + "/chairForceTorques.txt";

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
    contactLocationsFile.open(contactLocationsFilePath);

    l_footForceTorqueFile.open(l_footForceTorqueFilePath);
    r_footForceTorqueFile.open(r_footForceTorqueFilePath);
    chairForceTorqueFile.open(chairForceTorqueFilePath);


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
    ok &= contactLocationsFile.is_open();

    ok &= l_footForceTorqueFile.is_open();
    ok &= r_footForceTorqueFile.is_open();
    ok &= chairForceTorqueFile.is_open();
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
    contactLocationsFile.close();

    l_footForceTorqueFile.close();
    r_footForceTorqueFile.close();
    chairForceTorqueFile.close();
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

    Eigen::Vector3d l_sole_FrontLeft = l_sole_center + Eigen::Vector3d(0.08, 0.02, 0.0);
    Eigen::Vector3d l_sole_BackLeft = l_sole_center + Eigen::Vector3d(-0.02, 0.02, 0.0);

    // <task name="RightFootContact_BackRight" type="PointContact">
    // <offset x="-0.02" y=" 0.02" z="0.0" qw="0.0" qx="-0.707107" qy="-0.707107" qz="0.0" />
    //
    // <task name="RightFootContact_FrontRight" type="PointContact">
    // <offset x=" 0.06" y=" 0.02" z="0.0" qw="0.0" qx="-0.707107" qy="-0.707107" qz="0.0" />

    Eigen::Vector3d r_sole_center =  model->getSegmentPosition("r_sole").getTranslation();

    Eigen::Vector3d r_sole_FrontRight = r_sole_center + Eigen::Vector3d(0.08, -0.02, 0.0);
    Eigen::Vector3d r_sole_BackRight = r_sole_center + Eigen::Vector3d(-0.02, -0.02, 0.0);

    Eigen::Vector3d l_legContact = leftLegContactTask->getTaskState().getPosition().getTranslation();
    Eigen::Vector3d r_legContact = rightLegContactTask->getTaskState().getPosition().getTranslation();

    contactLocationsFile << l_sole_FrontLeft.transpose() << "\n";
    contactLocationsFile << l_sole_BackLeft.transpose() << "\n";
    contactLocationsFile << r_sole_FrontRight.transpose() << "\n";
    contactLocationsFile << r_sole_BackRight.transpose() << "\n";
    contactLocationsFile << l_legContact.transpose() << "\n";
    contactLocationsFile << r_legContact.transpose() << "\n";

    double x_min = std::fmax(l_legContact(0), r_legContact(0));
    double x_max = std::fmax(r_sole_FrontRight(0), l_sole_FrontLeft(0));

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


    yarp::os::Bottle *l_foot_btl = l_foot_FT_port.read(false);
    if ( (l_foot_btl!=NULL) && (l_foot_btl->size()==6) ) {
        l_footForceTorqueFile << relativeTime                  << "\t";
        l_footForceTorqueFile << l_foot_btl->get(0).asDouble() << "\t";
        l_footForceTorqueFile << l_foot_btl->get(1).asDouble() << "\t";
        l_footForceTorqueFile << l_foot_btl->get(2).asDouble() << "\t";
        l_footForceTorqueFile << l_foot_btl->get(3).asDouble() << "\t";
        l_footForceTorqueFile << l_foot_btl->get(4).asDouble() << "\t";
        l_footForceTorqueFile << l_foot_btl->get(5).asDouble() << "\n";
    }

    yarp::os::Bottle *r_foot_btl = r_foot_FT_port.read(false);
    if ( (r_foot_btl!=NULL) && (r_foot_btl->size()==6) ) {
        r_footForceTorqueFile << relativeTime                  << "\t";
        r_footForceTorqueFile << r_foot_btl->get(0).asDouble() << "\t";
        r_footForceTorqueFile << r_foot_btl->get(1).asDouble() << "\t";
        r_footForceTorqueFile << r_foot_btl->get(2).asDouble() << "\t";
        r_footForceTorqueFile << r_foot_btl->get(3).asDouble() << "\t";
        r_footForceTorqueFile << r_foot_btl->get(4).asDouble() << "\t";
        r_footForceTorqueFile << r_foot_btl->get(5).asDouble() << "\n";
    }

    yarp::os::Bottle *chair_btl = chair_FT_port.read(false);
    if ( (chair_btl!=NULL) && (chair_btl->size()==6) ) {
        chairForceTorqueFile << relativeTime                   << "\t";
        chairForceTorqueFile << chair_btl->get(0).asDouble() << "\t";
        chairForceTorqueFile << chair_btl->get(1).asDouble() << "\t";
        chairForceTorqueFile << chair_btl->get(2).asDouble() << "\t";
        chairForceTorqueFile << chair_btl->get(3).asDouble() << "\t";
        chairForceTorqueFile << chair_btl->get(4).asDouble() << "\t";
        chairForceTorqueFile << chair_btl->get(5).asDouble() << "\n";
    }

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


void StandClient::startRecording()
{
    yarp::os::Bottle message, reply;
    message.addString("record");
    message.addInt(1);
    message.addString(recordDir);
    message.addString(recordName);
    cameraPort.write(message, reply);
    if (reply.get(0).asBool()) {
        std::cout << "Recording started." << std::endl;
    } else {
        std::cout << "[ERROR] Failed to start simulation recording." << std::endl;
    }
    yarp::os::Time::delay(recordDelay);

    // message.clear();
    // message.addInt(1);
    // contactPort.write(message);
}

void StandClient::stopRecording()
{
    yarp::os::Time::delay(recordDelay);
    yarp::os::Bottle message, reply;
    message.addString("record");
    message.addInt(0);
    cameraPort.write(message, reply);
    if (reply.get(0).asBool()) {
        std::cout << "Recording stopped." << std::endl;
    } else {
        std::cout << "[ERROR] Failed to stop simulation recording." << std::endl;
    }

    // message.clear();
    // message.addInt(0);
    // contactPort.write(message);
}
