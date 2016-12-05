#ifndef TECH_TASK_ROBOTMAPPER_H
#define TECH_TASK_ROBOTMAPPER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "Controller.h"
#include "Navigator.h"
#include "SweepingPlanner.h"
#include <Communicator.h>

/**
 * Class that integrate all components of the planner and is responsible for generation of a mission for the robots.
 */
class RobotMapper : TaskReachedCallback, CommunicatorCallback {
public:

    RobotMapper(ros::NodeHandle &nh, const std::string &nameSpace);

    /** Inform from planner that the mission was finished */
    void taskReached();

    /** Inform from communicator that the mission was received */
    void missionArrived();


private:

    ros::NodeHandle nodeH;
    std::string nameSpace;

    geometry_msgs::PoseConstPtr ownPose;

    /** subscribe own state */
    ros::Subscriber subRobotState, mapSubscriber;

    bool amIMaster = false;

    double robotRadius;
    double goalX, goalY;

    //----- COMPONENTS -------//
    std::shared_ptr<Controller> controller;
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<SweepingPlanner> sweepPlanner;
    std::shared_ptr<Communicator> communicator;
    std::shared_ptr<Map> map;


    //----- METHODS  -------//

    void initComponents();

    void estimateMasterAndStart();

    /** save robot's own position */
    void ownStateCallback(const geometry_msgs::PoseConstPtr &ownPose);

    void startMission();

    void mapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &mapMsg);

    /** topic used for position subscription */
    std::string positionSubscribeTopic;
    /** topic used for velocity publising*/
    std::string velocityTopic;

    void passMissionToPlanner(Area area, Position goal);

    void assignSweepArea();

    void assignAreasToRobots(std::vector<Area> areas, std::unordered_map<std::string, Position> ngbrPoses,
                             std::unordered_map<std::string, Area> &mission);

    void findClosestRemaining(const std::unordered_map<std::string, Position> &ngbrPoses,
                              const std::vector<std::string> &alreadyAssigned,
                              Position areaStart, std::string &closestRobot);
};

#endif //TECH_TASK_ROBOTMAPPER_H
