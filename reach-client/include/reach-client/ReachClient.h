#ifndef REACHCLIENT_H
#define REACHCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
#include <list>
#include <fstream>

#include <ocra/util/StringUtilities.h>

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

private:
    // Waypoint files
    std::string rightHandWaypointFilePath;
    std::string comWaypointFilePath;

    // Waypoint lists
    std::list<Eigen::VectorXd> rightHandWaypointList;
    std::list<Eigen::VectorXd> comWaypointList;

    // Trajectory threads
    ocra_recipes::TrajectoryThread::Ptr rightHandTrajThread;
    ocra_recipes::TrajectoryThread::Ptr comTrajThread;

    // Loop logic variables.
    bool trigger;
    double startTime;
    double LOOP_TIME_LIMIT;
    double relativeTime;

    // Logging variables
    Eigen::Vector3d rightHandGoalPosition;
    Eigen::Vector3d comGoalPosition;
    Eigen::Vector3d rightHandPosition;
    Eigen::Vector3d comPosition;
    Eigen::VectorXd torques;

};


#endif // TEST_CLIENT_H
