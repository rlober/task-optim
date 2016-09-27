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

bool ReachClient::createDataFiles()
{
    rightHandPositionRealFilePath = savePath + "/rightHandPositionReal.txt";
    rightHandPositionRefFilePath = savePath + "/rightHandPositionRef.txt";
    comPositionRealFilePath = savePath + "/comPositionReal.txt";
    comPositionRefFilePath = savePath + "/comPositionRef.txt";
    torquesFilePath = savePath + "/torques.txt";
    rightHandPositionRealFile.open(rightHandPositionRealFilePath);
    rightHandPositionRefFile.open(rightHandPositionRefFilePath);
    comPositionRealFile.open(comPositionRealFilePath);
    comPositionRefFile.open(comPositionRefFilePath);
    torquesFile.open(torquesFilePath);

    bool ok = true;
    ok &= rightHandPositionRealFile.is_open();
    ok &= rightHandPositionRefFile.is_open();
    ok &= comPositionRealFile.is_open();
    ok &= comPositionRefFile.is_open();
    ok &= torquesFile.is_open();
    return ok;
}


void ReachClient::closeDataFiles()
{
    rightHandPositionRealFile.close();
    rightHandPositionRefFile.close();
    comPositionRealFile.close();
    comPositionRefFile.close();
    torquesFile.close();
}

bool ReachClient::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("rightHandWptFile") && rf.check("comWptFile") && rf.check("savePath") ) {
        rightHandWaypointFilePath = rf.find("rightHandWptFile").asString().c_str();
        comWaypointFilePath = rf.find("comWptFile").asString().c_str();
        savePath = rf.find("savePath").asString().c_str();
        rightHandWaypointFilePath = boost::filesystem::canonical(rightHandWaypointFilePath).string();
        comWaypointFilePath = boost::filesystem::canonical(comWaypointFilePath).string();
        savePath = boost::filesystem::canonical(savePath).string();

        std::cout << "rightHandWaypointFilePath: \n" << rightHandWaypointFilePath << std::endl;
        std::cout << "comWaypointFilePath: \n" << comWaypointFilePath << std::endl;
        std::cout << "savePath: \n" << savePath << std::endl;
        bool ok = getWaypointDataFromFile(rightHandWaypointFilePath, rightHandWaypointList);
        ok &= getWaypointDataFromFile(comWaypointFilePath, comWaypointList);
        ok &= createDataFiles();
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
    rightHandTask = std::make_shared<ocra_recipes::TaskConnection>("RightHandCartesian");
    comTask = std::make_shared<ocra_recipes::TaskConnection>("ComTask");

    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::TIME_OPTIMAL;
    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::CYCLE;
    rightHandTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RightHandCartesian", rightHandWaypointList, trajType, termStrategy);
    rightHandTrajThread->setGoalErrorThreshold(0.03);



    termStrategy = ocra_recipes::WAIT;
    comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", comWaypointList, trajType, termStrategy);
    comTrajThread->start();
    rightHandTrajThread->start();
    startTime = yarp::os::Time::now();
    return true;
}

void ReachClient::release()
{
    if(rightHandTrajThread){rightHandTrajThread->stop();}
    if(comTrajThread){comTrajThread->stop();}
    closeDataFiles();
}

void ReachClient::loop()
{
    relativeTime = yarp::os::Time::now() - startTime;

    if (rightHandTrajThread->goalAttained()) {
        std::cout << "Attained Goal. Stopping." << std::endl;
        stop();
    }

    if (relativeTime > LOOP_TIME_LIMIT) {
        std::cout << "Loop time limit exceeded. Stopping." << std::endl;
        stop();
    }

    logClientData();

}

void ReachClient::logClientData()
{
    rightHandPositionRefFile << rightHandTask->getDesiredTaskState().getPosition().getTranslation().transpose() << "\n";
    rightHandPositionRealFile << model->getSegmentPosition("r_hand").getTranslation().transpose() << "\n";
    comPositionRefFile << comTask->getDesiredTaskState().getPosition().getTranslation().transpose() << "\n";
    comPositionRealFile << model->getCoMPosition().transpose() << "\n";
    torquesFile << model->getJointTorques().transpose() << "\n";
}
