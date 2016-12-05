#ifndef TECH_TASK_NAVIGATOR_H
#define TECH_TASK_NAVIGATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <unordered_set>
#include "TaskReachedCallback.h"
#include "Controller.h"
#include "Map.h"
#include "GraphLocation.h"
#include "SearchGraph.h"
#include "unordered_map"
#include <queue>
#include <unordered_map>


enum NavState{
    AVOIDING,           // avoiding neighbors
    NAVIGATING,         // checking execution
    WAITING_FOR_DATA    // waiting for next missionWPT or plan to be created
};

const double DEFAULT_ROBOT_RADIUS_M = 0.4;

/**
 * Handle path planning around obstacles and avoidance of higher priority members
 */
class Navigator : public TaskReachedCallback {
public:
    Navigator(ros::NodeHandle &nh, const std::string &nameSpace, const std::string positionSubscribeTopic, std::shared_ptr<TaskReachedCallback> owner);


    void addController(std::shared_ptr<Controller> controller);

    void setMap(std::shared_ptr<Map> &map);


    /** Set actual mission waypoint*/
    void planPathTo(Position & missionWpt);

    /**
     * Update position of a neighbor
     * @param nbrName
     * @param pose
     */
    void updateNbrPosition(std::string nbrName, const Position &pose);

    /**
     * Update current waypoint of a neighbor
     * @param nbrName
     * @param wpt
     */
    void updateNbrWpts(std::string nbrName, const Position &wpt);

    /**
     * Return current mission waypoint
     * @param currentMissionWpt
     */
    void getCurrentWaypoint(Position &currentMissionWpt);


    /**
     * Upload 1 wpt at a time to controller and inform owner when finished
     */
    void taskReached();

    /**
     * Inform robot about finished neighbor, avoid its position from now on
     * @param finishedRobot
     * @param robotsFinalPose
     */
    void neighborFinished(const std::string finishedRobot, const Position robotsFinalPose);

    double getRobotRadius();


private:
    ros::NodeHandle nodeH;
    std::string nameSpace;

    geometry_msgs::Pose ownPose;
    Position currentMissionWpt;

    /** subscribe own state and map */
    ros::Subscriber subRobotState, mapSubscriber;

    NavState state;
    bool currWptSet = false;
    bool ownPoseSet = false;
    bool mapSet = false;

    /** remaining waypoints to visit */
    std::vector<GraphLocation> wptPath;
    std::deque<GraphLocation> wptQueue;

    /** radius of a robot for collision handling */
    double robotRadius;

    //----- MUTEXES  -------//
    boost::shared_mutex mutex_pos_;
    boost::shared_mutex mutex_plans_;

    //----- NEIGHBORS -------//
    // higher priority neighbors data
    std::unordered_set<std::string> nghbrNames;
    std::unordered_map<std::string, Position> neighborPoses;
    std::unordered_map<std::string, Position> neighborPlans;
    std::unordered_map<std::string, Position> finishedNeighbors;

    //----- COMPONENTS -------//
    std::shared_ptr<Controller> controller;
    std::shared_ptr<SearchGraph> graph;
    std::shared_ptr<Map> map;

    /** Mission planner (SweepingPlanner) */
    std::shared_ptr<TaskReachedCallback> owner;

    //----- CALLBACK FUNCTIONS -------//

    /** save robot's own position */
    void ownStateCallback(const geometry_msgs::PoseConstPtr &ownPose);

    /** Timer for checking state */
    ros::Timer timerLoop;

    void checkStateLoop(const ros::TimerEvent &);

    void changeState(const NavState &newState);

    void plan();

    void checkExecution();

    /**
     * Plan A* trajectory
     * @param graph
     * @param start
     * @param goal
     * @param came_from
     * @param cost_so_far
     * @return
     */
    bool aStarSearch (std::shared_ptr<SearchGraph> graph, GraphLocation start, GraphLocation &goal, std::unordered_map<GraphLocation, GraphLocation>& came_from,
                                std::unordered_map<GraphLocation, double>& cost_so_far);

    /*
     * Heuristic function for A*
     */
    double heuristic(GraphLocation location, GraphLocation goal);


    /**
     * Reconstruct path from map
     * @param start
     * @param goal
     * @param cameFrom
     * @param path
     */
    void reconstructPath(GraphLocation start, GraphLocation goal,
                         std::unordered_map<GraphLocation, GraphLocation> &cameFrom, std::deque<GraphLocation> &path);


    void printPath(std::deque<GraphLocation> vector);

    /**
     * Upload waypoint to controller
     * @param location
     */
    void uploadWptToController(GraphLocation &location);

    /**
     * Find too near neighbors
     * @param currentPos
     * @param nearNbrs
     */
    void checkNearNgbrs(const Position &currentPos, std::vector<std::string> &nearNbrs);

    void handleNearNgbrs(const Position curPos, const std::vector<std::string> &nearNgbrs);

    void checkAvoidance();

    void returnToPlan();


    void countAvoidancePushDirection(const Position curPosition, const std::vector<std::string> &nearNeighbors,
                                                const std::vector<Position> &nearObstacles, Position &push);

    double getPushAmplitude(double robotCoord, double obstacleCoord);
};

#endif //TECH_TASK_NAVIGATOR_H
