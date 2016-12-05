#ifndef TECH_TASK_COMMUNICATOR_H
#define TECH_TASK_COMMUNICATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include "unordered_set"
#include "unordered_map"
#include "Map.h"
#include "Navigator.h"
#include "SweepingPlanner.h"
#include "CommunicatorCallback.h"

/**
 * Handle interation with other robots - exchange positions and waypoints
 */
class Communicator {
public:
    Communicator(ros::NodeHandle &nh, const std::string &nameSpace,
                 const std::string positionSubscribeTopic, CommunicatorCallback* owner);

    void addNavigator(std::shared_ptr<Navigator> navigator);

    /**
     * @return true if there are no robots with higher priority
     */
    bool amIMaster();

    /**
     * Return map of neighbor robot names and their positions
     * @param neighborPoses
     */
    void getNgbrPoses(std::unordered_map<std::string, Position> &neighborPoses);

    /**
     * Distribute mission to the followers.
     * @param missions
     */
    void distributeMission(const std::unordered_map<std::string, Area> &missions);

    /**
     * Get own mission.
     * Mission consists of 2 Positions, which define sweep area
     * @param mission
     */
    void getMissionData(std::vector<Position> &mission);

    /**
     * Inform about finished mission of the robot, other robots need to avoid it now
     */
    void informMissionFinished();


private:
    ros::NodeHandle nodeH;
    std::string nameSpace;

    ros::Subscriber subRobotPose, subOtherNames, subMission;
    ros::Publisher pubOwnName, pubPlan, pubMission;
    std::vector<ros::Subscriber> otherStatesSubscribers, plansSubscribers;

    const std::string PLAN_SUFFIX = "/plans";
    const std::string positionSubscribeTopic;

    ros::Timer timerName, timerPlan;

    geometry_msgs::Pose ownPose;
    Area area;
    Position goal;
    bool ownPoseArrived = false;
    bool master = true;


    /** neighbors */
    std::unordered_set<std::string> neighborNames;
    std::unordered_set<std::string> higherPriorityNghbrs;
    std::unordered_map<std::string, Position> neighborPoses;
    std::unordered_map<std::string, Position> neighborPlans;

    //----- COMPONENTS -------//

    /** mission waypoint navigator - inform it about neighbors */
    std::shared_ptr<Navigator> navigator;
    CommunicatorCallback* owner;


    /** Robot pose callback function which saves current robot pose */
    void ownStateCallback(const geometry_msgs::PoseConstPtr &msg);


    void processNeighborsCallback(const std_msgs::String::ConstPtr &nbrName);

    void handleNewNeighbor(std::string &name);

    void subscribeNbrPos(std::string &neighborName);

    void processNeighborState(const geometry_msgs::PoseConstPtr &msg, std::string &neighborName);

    void informNameLoop(const ros::TimerEvent &);

    void broadcastPlanLoop(const ros::TimerEvent &);

    void insertHighPriorityNgbr(std::string ngbrName);

    void processNgbrPlan(const nav_msgs::Path::ConstPtr &msg);

    void missionArrived(const nav_msgs::PathConstPtr &mission);

};

#endif //TECH_TASK_COMMUNICATOR_H
