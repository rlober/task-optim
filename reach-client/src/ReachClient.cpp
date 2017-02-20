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

    rightHandJacobiansFilePath = savePath + "/rightHandJacobians.txt";
    comJacobiansFilePath = savePath + "/comJacobians.txt";
    jointPositionsFilePath = savePath + "/jointPositions.txt";
    jointLimitsFilePath = savePath + "/jointLimits.txt";
    attainedGoalFilePath = savePath + "/attainedGoal.txt";

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

    rightHandJacobiansFile.open(rightHandJacobiansFilePath);
    comJacobiansFile.open(comJacobiansFilePath);
    jointPositionsFile.open(jointPositionsFilePath);
    jointLimitsFile.open(jointLimitsFilePath);
    attainedGoalFile.open(attainedGoalFilePath);


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
    ok &= rightHandJacobiansFile.is_open();
    ok &= comJacobiansFile.is_open();
    ok &= jointPositionsFile.is_open();
    ok &= jointLimitsFile.is_open();
    ok &= attainedGoalFile.is_open();
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
    rightHandJacobiansFile.close();
    comJacobiansFile.close();
    jointPositionsFile.close();
    jointLimitsFile.close();
    attainedGoalFile.close();
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
                recordName = "reaching";
            }

            if (rf.check("recordDelay")) {
                recordDelay = rf.find("recordDelay").asDouble();
            } else {
                recordDelay = 0.0;
            }

            std::cout << "recordDir: " << recordDir << std::endl;
            std::cout << "recordName: " << recordName << std::endl;
            std::cout << "recordDelay: " << recordDelay << std::endl;

            std::string cameraPortName("/ReachClient/camera/rpc:o");
            cameraPort.open(cameraPortName);
            yarp.connect(cameraPortName, "/Gazebo/yarp_camera_sensor/rpc:i");

        } else {
            recordSimulation = false;
        }

        sentVisPortMessage = false;
        usingGazeboSim = yarp.exists("/Gazebo/OcraGuiPlugin/rpc:i");
        // usingGazeboSim = yarp.exists("/Gazebo/RightHandTarget:i");
        // usingGazeboSim &= yarp.exists("/Gazebo/TaskOptim/RightHandTarget/change_color:i");
        if (usingGazeboSim) {
            if (rf.check("case")) {
                case_label = rf.find("case").asString().c_str();
            } else {
                std::cout << "[ERROR] You have to provide a case label!!!!" << std::endl;
                case_label = "unknown";
            }
            std::string gazeboRpcPortName("/ReachClient/gazebo/rpc:o");
            gazeboRpcPort.open(gazeboRpcPortName);
            yarp.connect(gazeboRpcPortName, "/Gazebo/OcraGuiPlugin/rpc:i");
            // std::string posPortName("/ReachClient/target/position:o");
            // std::string visPortName("/ReachClient/target/visual:o");
            // posPort.open(posPortName);
            // visPort.open(visPortName);
            // yarp.connect(posPortName, "/Gazebo/RightHandTarget:i");
            // yarp.connect(visPortName, "/Gazebo/TaskOptim/RightHandTarget/change_color:i");
        } else {
            std::cout << "\n\n\nUsing real robot.\n\n" << std::endl;
        }



        if (ok) {
            rightHandGoalPosition = *rightHandWaypointList.rbegin();
            comGoalPosition = *comWaypointList.rbegin();
            rightHandSegmentName = "r_hand";

            return true;
        } else {
            return false;
        }

    } else {
        return false;
    }
}

std::string ReachClient::getSdfColorString(const std::string& label)
{
    std::string rgbString;
    if (label == "reachable") {
        rgbString = "0.192156862745 0.639215686275 0.329411764706";
    } else if (label=="possibly_reachable") {
        rgbString = "0.901960784314 0.333333333333 0.0509803921569";
    } else if (label=="unreachable") {
        rgbString = "0.870588235294 0.176470588235 0.149019607843";
    } else {
        rgbString = "0.1 0.1 0.1";
    }
    std::string colorString =   "<ambient> " + rgbString +" 0.9 </ambient>\
                                <diffuse> " + rgbString +" 0.9 </diffuse>\
                                <specular> " + rgbString +" 0.9 </specular>\
                                <emissive> " + rgbString +" 0.9 </emissive>\n";
    return colorString;
}

std::string ReachClient::getSdf(const Eigen::Vector3d& pose, const std::string& label)
{
    std::string color = getSdfColorString(label);
    std::string pose_string = std::to_string(pose(0))+" "+std::to_string(pose(1))+" "+std::to_string(pose(2))+" 0 0 0";
    std::string sdf_string = "<?xml version='1.0' ?>\
    <sdf version='1.5'>\
      <model name='right_hand_target'>\
        <static>true</static>\
        <link name='link'>\
          <visual name='visual'>\
            <pose>"+pose_string+"</pose>\
            <geometry>\
              <sphere>\
                  <radius>0.03</radius>\
              </sphere>\
            </geometry>\
            <material>"+color+"</material>\
            <transparency>0.2</transparency>\
          </visual>\
        </link>\
      </model>\
    </sdf>";
    return sdf_string;
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
        rightHandTermStrategy = ocra_recipes::TERMINATION_STRATEGY::WAIT;
        comTermStrategy = ocra_recipes::TERMINATION_STRATEGY::NONE;
    }


    rightHandTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RightHandCartesian", rightHandWaypointList, trajType, rightHandTermStrategy);

    rightHandTrajThread->setGoalErrorThreshold(0.03);


    if (usingComTask) {
        comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", comWaypointList, trajType, comTermStrategy);

        // comTrajThread->setMaxVelocityAndAcceleration(0.005, 0.005);

    }
    rightHandTask = std::make_shared<ocra_recipes::TaskConnection>("RightHandCartesian");
    comTask = std::make_shared<ocra_recipes::TaskConnection>("ComTask");

    std::cout << "-------------------------------------------------------------------" << std::endl;
    std::cout << "Right Hand" << std::endl;
    std::cout << "-------------------------------------------------------------------" << std::endl;
    std::cout << "Starting position: " <<  rightHandTask->getTaskState().getPosition().getTranslation().transpose()  << std::endl;
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

    initialRightHandState = rightHandTask->getDesiredTaskState();
    initialComState = comTask->getDesiredTaskState();

    getComBounds();
    getJointLimits();

    if (usingGazeboSim) {
        yarp::os::Bottle message, reply;
        message.addString("getRobotWorldPose");
        gazeboRpcPort.write(message, reply);
        Eigen::Vector3d gazebo_offset(reply.get(0).asDouble(), reply.get(1).asDouble(), reply.get(2).asDouble());
        std::cout << "gazebo_offset = " << gazebo_offset.transpose() << std::endl;
        Eigen::Vector3d rh_target_gazebo = rightHandGoalPosition + gazebo_offset;

        std::string sdf_string = getSdf(rh_target_gazebo, case_label);

        message.clear();
        reply.clear();

        message.addString("addSdfToWorld");
        message.addString(sdf_string);

        gazeboRpcPort.write(message, reply);



        // Eigen::Vector3d l_sole_center(0.0, 0.135, 0.0);// = model->getSegmentPosition("l_sole").getTranslation();
        // Eigen::Vector3d rh_target_gazebo = rightHandGoalPosition + l_sole_center;
        //
        // posBottle.addDouble(rh_target_gazebo(0));
        // posBottle.addDouble(rh_target_gazebo(1));
        // posBottle.addDouble(rh_target_gazebo(2));
    }
    if (recordSimulation) {
        startRecording();
    }
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

    Eigen::Vector3d l_sole_FrontLeft = l_sole_center + Eigen::Vector3d(0.06, 0.02, 0.0);
    Eigen::Vector3d l_sole_BackLeft = l_sole_center + Eigen::Vector3d(-0.02, 0.02, 0.0);

    // <task name="RightFootContact_BackRight" type="PointContact">
    // <offset x="-0.02" y=" 0.02" z="0.0" qw="0.0" qx="-0.707107" qy="-0.707107" qz="0.0" />
    //
    // <task name="RightFootContact_FrontRight" type="PointContact">
    // <offset x=" 0.06" y=" 0.02" z="0.0" qw="0.0" qx="-0.707107" qy="-0.707107" qz="0.0" />

    Eigen::Vector3d r_sole_center =  model->getSegmentPosition("r_sole").getTranslation();

    Eigen::Vector3d r_sole_FrontRight = r_sole_center + Eigen::Vector3d(0.06, -0.02, 0.0);
    Eigen::Vector3d r_sole_BackRight = r_sole_center + Eigen::Vector3d(-0.02, -0.02, 0.0);

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

void ReachClient::getJointLimits()
{
    jointLimitsFile << model->getJointLowerLimits().transpose() << "\n";
    jointLimitsFile << model->getJointUpperLimits().transpose() << "\n";
}

void ReachClient::release()
{

    if (logData) {
        writeWaypointsToFile();
        closeDataFiles();
    }

    if (recordSimulation) {
        stopRecording();
        cameraPort.close();
    }
    if (usingGazeboSim) {
        gazeboRpcPort.close();
        // posPort.close();
        // visPort.close();
    }
    if(rightHandTrajThread){rightHandTrajThread->stop();}
    if (usingComTask) {
        if(comTrajThread){comTrajThread->stop();}
    }
    rightHandTask->setDesiredTaskState(initialRightHandState);
    comTask->setDesiredTaskState(initialComState);
}

void ReachClient::changeTargetColor()
{
    // if (usingGazeboSim && !sentVisPortMessage) {
    //     yarp::os::Bottle b;
    //     b.addInt(1);
    //     visPort.write(b);
    //     sentVisPortMessage = true;
    // }
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
    // if (usingGazeboSim) {
    //     posPort.write(posBottle);
    // }
    relativeTime = yarp::os::Time::now() - startTime;

    if (rightHandTrajThread->goalAttained() || (rightHandTrajThread->isReturningHome() && !returningHome) ) {


        if (!goToHomeOnRelease) {
            std::cout << "Attained Goal. Stopping." << std::endl;
            changeTargetColor();
            attainedGoalFile << "1\n";
            stop();
        } else if (returningHome) {
            stop();
        } else {
            std::cout << "Attained Goal. Returning to home position." << std::endl;
            attainedGoalFile << "1\n";
            changeTargetColor();
            returningHome = true;
            if (usingComTask) {
                comTrajThread->returnToHome();
            }
        }
    }

    if (relativeTime > LOOP_TIME_LIMIT) {
        if (!goToHomeOnRelease) {
            std::cout << "Loop time limit exceeded. Stopping." << std::endl;
            attainedGoalFile << "0\n";
            stop();
        } else {
            if (!returningHome) {
                std::cout << "Loop time limit exceeded. Returning to home position." << std::endl;
                attainedGoalFile << "0\n";
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
    rightHandPositionRealFile << rightHandTask->getTaskState().getPosition().getTranslation().transpose() << "\n";
    // rightHandPositionRealFile << model->getSegmentPosition("r_hand").getTranslation().transpose() << "\n";
    comPositionRefFile << comTask->getDesiredTaskState().getPosition().getTranslation().transpose() << "\n";
    comPositionRealFile << model->getCoMPosition().transpose() << "\n";
    torquesFile << model->getJointTorques().transpose() << "\n";

    rightHandJacobian = model->getSegmentJacobian(rightHandSegmentName);
    comJacobian = model->getCoMJacobian();
    rightHandJacobiansFile << Eigen::VectorXd( Eigen::Map<Eigen::VectorXd>(rightHandJacobian.data(), rightHandJacobian.rows()*rightHandJacobian.cols()) ).transpose() << "\n";
    comJacobiansFile << Eigen::VectorXd( Eigen::Map<Eigen::VectorXd>(comJacobian.data(), comJacobian.rows()*comJacobian.cols()) ).transpose() << "\n";

    jointPositionsFile << model->getJointPositions().transpose() << "\n";
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
    double limitFactor = 2.5;
    if (usingComTask) {
        double comExpectedDuration = comTrajThread->getDuration();
        comExpectedDurationFile << comExpectedDuration;
        // Make time limit 3x the longest traj.
        LOOP_TIME_LIMIT = std::fmax(comExpectedDuration, rightHandExpectedDuration)*limitFactor;
    } else {
        LOOP_TIME_LIMIT = rightHandExpectedDuration*limitFactor;
    }
}


void ReachClient::startRecording()
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
}

void ReachClient::stopRecording()
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
}
