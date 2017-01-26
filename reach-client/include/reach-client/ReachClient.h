#ifndef REACHCLIENT_H
#define REACHCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
#include <list>
#include <fstream>

#include <ocra/util/StringUtilities.h>

#include <yarp/os/all.h>

class ReachClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(ReachClient)

public:
    ReachClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~ReachClient ();

    virtual bool configure(yarp::os::ResourceFinder &rf);


protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    bool getWaypointDataFromFile(const std::string& filePath, std::list<Eigen::VectorXd>& waypointList);
    void logClientData();
    bool createDataFiles();
    void closeDataFiles();
    void writeWaypointsToFile();
    void setLoopTimeLimit();
    void getComBounds();
    void getJointLimits();

    void startRecording();
    void stopRecording();
    void changeTargetColor();




private:
    // Waypoint files
    std::string rightHandWaypointFilePath;
    std::string comWaypointFilePath;
    std::string savePath;
    std::string rightHandPositionRealFilePath;
    std::string rightHandPositionRefFilePath;
    std::string comPositionRealFilePath;
    std::string comPositionRefFilePath;
    std::string torquesFilePath;
    std::string comWaypointsFilePath;
    std::string rightHandWaypointsFilePath;
    std::string timelineFilePath;
    std::string comExpectedDurationFilePath;
    std::string rightHandExpectedDurationFilePath;
    std::string comBoundsFilePath;
    std::string rightHandJacobiansFilePath;
    std::string comJacobiansFilePath;
    std::string jointPositionsFilePath;
    std::string jointLimitsFilePath;
    std::string attainedGoalFilePath;

    std::ofstream rightHandPositionRealFile;
    std::ofstream rightHandPositionRefFile;
    std::ofstream comPositionRealFile;
    std::ofstream comPositionRefFile;
    std::ofstream torquesFile;
    std::ofstream comWaypointsFile;
    std::ofstream rightHandWaypointsFile;
    std::ofstream timelineFile;
    std::ofstream comExpectedDurationFile;
    std::ofstream rightHandExpectedDurationFile;
    std::ofstream comBoundsFile;
    std::ofstream rightHandJacobiansFile;
    std::ofstream comJacobiansFile;
    std::ofstream jointPositionsFile;
    std::ofstream jointLimitsFile;
    std::ofstream attainedGoalFile;

    // Waypoint lists
    std::list<Eigen::VectorXd> rightHandWaypointList;
    std::list<Eigen::VectorXd> comWaypointList;

    // Jacobians
    Eigen::MatrixXd rightHandJacobian;
    Eigen::MatrixXd comJacobian;

    // Trajectory threads
    ocra_recipes::TrajectoryThread::Ptr rightHandTrajThread;
    ocra_recipes::TrajectoryThread::Ptr comTrajThread;

    // Loop logic variables.
    bool trigger;
    bool logData;
    double startTime;
    double LOOP_TIME_LIMIT;
    double relativeTime;

    // Logging variables
    Eigen::Vector3d rightHandGoalPosition;
    Eigen::Vector3d comGoalPosition;
    Eigen::Vector3d rightHandPosition;
    Eigen::Vector3d comPosition;
    Eigen::VectorXd torques;

    ocra_recipes::TaskConnection::Ptr rightHandTask;
    ocra_recipes::TaskConnection::Ptr comTask;

    bool goToHomeOnRelease;
    ocra_recipes::TERMINATION_STRATEGY rightHandTermStrategy;
    ocra_recipes::TERMINATION_STRATEGY comTermStrategy;
    bool returningHome;
    bool usingComTask;

    ocra::TaskState initialRightHandState;
    ocra::TaskState initialComState;

    std::string rightHandSegmentName;

    yarp::os::RpcClient cameraPort;
    yarp::os::Network yarp;
    bool recordSimulation;
    std::string recordDir;
    std::string recordName;
    double recordDelay;

    yarp::os::Port posPort;
    yarp::os::Port visPort;
    bool usingGazeboSim;
    bool sentVisPortMessage;
    yarp::os::Bottle posBottle;


};


#endif // TEST_CLIENT_H
