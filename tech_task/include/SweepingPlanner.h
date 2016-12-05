#ifndef TECH_TASK_SWEEPINGPLANNER_H
#define TECH_TASK_SWEEPINGPLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "TaskReachedCallback.h"
#include "Navigator.h"

struct Area{

    Position start;
    Position end;

    Area(Position s, Position e): start(s), end(e){
    }

    Area():start(0,0), end(0,0){
    }
};

struct Pattern{
    int *x;
    int *y;

    Pattern(int *xPat, int *yPat){
        x = xPat;
        y = yPat;
    }
};

/**
 * Create sweeping pattern around obstacles
 */
class SweepingPlanner: public TaskReachedCallback{
public:

    SweepingPlanner(ros::NodeHandle &nh, const std::string &name_space,
                    const std::string positionSubscribeTopic, TaskReachedCallback *owner);

    /** Upload mission waypoints one at a time */
    void taskReached();

    void addNavigator(std::shared_ptr<Navigator> navigator);

    void setMap(std::shared_ptr<Map> &map);

    /**
     * Define sweeping area and final goal position
     * @param area
     * @param goalPosition
     */
    void sweepArea(const Area &area, const Position &goalPosition);

    /**
     * Divide area in a way to minimize number of sweep turns, according to number of robots in team
     * @param corners - area corners
     * @param parts - number of robots
     * @param areas
     */
    void divideArea(std::vector<Position> corners, int parts, std::vector<Area> &areas);

private:

    ros::NodeHandle nodeH;
    std::string nameSpace;

    geometry_msgs::Pose ownPose;

    /** Inform about finished mission (RobotMapper) */
    TaskReachedCallback *owner;

    /** subscribe own state */
    ros::Subscriber subRobotState;
    ros::Publisher marker_pub;

    ros::Timer dataCheckLoop;

    bool mapArrived = false;
    bool ownPoseArrived = false;
    bool areaSet = false;
    bool planning = false;

    int posCounter = 0;
    std::vector<Position> path;

    Area areaToSweep;
    Position goal;
    std::vector<Position> remainingMission;

    //----- COMPONENTS -------//

    /** mission waypoint navigator */
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<Map> map;


    /** save robot's own position */
    void ownStateCallback(const geometry_msgs::PoseConstPtr &ownPose);

    /**
     * Create sweeping zig-zag pattern
     * @param vector - vector of zig-zag mission waypoints
     */
    void createSweepPattern(std::vector<Position> &vector);

    /**
     * Find closest and most distant corners to own position
     * @param position
     * @param dist
     * @param corners
     */
    void findCloseDist(Position &position, Position &dist, const std::vector<Position> & corners);

    void checkStateLoop(const ros::TimerEvent &);

    /**
     * Create sweep pattern waypoints
     * @param closestP - closest area corner from the robot
     * @param mostDistP  - most distant area corner from the robot
     * @param useHorizontal - horizontal/vertical pattern
     * @param waypoints - returned waypoints
     */
    void getSweepPattern(const Position &closestP, const Position &mostDistP, bool useHorizontal,
                         std::vector<Position> &waypoints) const;


    void visualizePattern(std::vector<Position> &waypoints);

    void visualizeArea(std::vector<Position> vector);

    void createCornerVector(Area area, std::vector<Position> &vec);

    void visualizePath(const Position &pos);
};
#endif //TECH_TASK_SWEEPINGPLANNER_H
