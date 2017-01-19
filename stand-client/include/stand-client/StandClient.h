#ifndef STANDCLIENT_H
#define STANDCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
#include <list>
#include <fstream>

#include <ocra/util/StringUtilities.h>

#include <yarp/os/all.h>


class StandClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(StandClient)

public:
    StandClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~StandClient ();
    virtual bool configure(yarp::os::ResourceFinder &rf);


protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();
    // virtual void printHelp();

private:
    bool getWaypointDataFromFile(const std::string& filePath, std::list<Eigen::VectorXd>& waypointList);
    void logClientData();
    bool createDataFiles();
    void closeDataFiles();
    void writeWaypointsToFile();
    void setLoopTimeLimit();
    void getComBounds();
    void getJointLimits();
    void deactivateLegContacts();

    bool getComWaypoints();
    Eigen::Vector3d getComGoalWaypoint();
    std::list<Eigen::VectorXd> getManualSolutionWaypoints();

    void startRecording();
    void stopRecording();



private:
    yarp::os::ResourceFinder _rf;

    std::string comWaypointFilePath;
    std::string savePath;
    std::string comPositionRealFilePath;
    std::string comPositionRefFilePath;
    std::string torquesFilePath;
    std::string comWaypointsFilePath;
    std::string timelineFilePath;
    std::string comExpectedDurationFilePath;
    std::string comBoundsFilePath;
    std::string comJacobiansFilePath;
    std::string jointPositionsFilePath;
    std::string jointLimitsFilePath;

    std::ofstream comPositionRealFile;
    std::ofstream comPositionRefFile;
    std::ofstream torquesFile;
    std::ofstream comWaypointsFile;
    std::ofstream timelineFile;
    std::ofstream comExpectedDurationFile;
    std::ofstream comBoundsFile;
    std::ofstream comJacobiansFile;
    std::ofstream jointPositionsFile;
    std::ofstream jointLimitsFile;

    // Waypoint lists
    std::list<Eigen::VectorXd> comWaypointList;

    // Jacobians
    Eigen::MatrixXd comJacobian;

    // Trajectory threads
    ocra_recipes::TrajectoryThread::Ptr comTrajThread;

    // Com task
    ocra_recipes::TaskConnection::Ptr comTask;

    // For the seat contacts
    ocra_recipes::TaskConnection::Ptr leftLegContactTask;
    ocra_recipes::TaskConnection::Ptr rightLegContactTask;

    // Loop logic variables.
    bool logData;
    double startTime;
    double LOOP_TIME_LIMIT;
    double relativeTime;
    double contactReleaseDelay;
    bool contactsReleased;

    yarp::os::RpcClient cameraPort;
    yarp::os::Network yarp;
    bool recordSimulation;
    std::string recordDir;
    std::string recordName;
    double recordDelay;

};


#endif // TEST_CLIENT_H
