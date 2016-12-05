#ifndef TECH_TASK_CONTROLLER_H
#define TECH_TASK_CONTROLLER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <utils.h>
#include <angles/angles.h>
#include <math.h>
#include <TaskReachedCallback.h>
#include <boost/thread/thread.hpp>


enum State {OFF, STAND_BY, REACHING_HEADING, NAV_TO_WPT};


/**
 * Waypoint chase controller that does not take obstacles into account
 */
class Controller{
public:
    Controller(ros::NodeHandle &nh, const std::string &nameSpace, const std::string positionSubscribeTopic,
               const std::string velocityTopic, std::shared_ptr<TaskReachedCallback> owner);

    /** Robot pose callback function which saves current robot pose */
    void ownStateCallback(const geometry_msgs::PoseConstPtr &msg);


    /** Stops the robot */
    void stop_robot();


    /** Moves robot based on simple sontrol algorithm  */
    void move();

    /**
     * Publishes velocity to given topic
     */
    void publish_velocity();


    int setTargetWaypoint(const geometry_msgs::Point &point);

    /** return true if close to target and stop robot */
    bool isTargetReached();

    /** check control state machine */
    void checkStateLoop(const ros::TimerEvent &);

    /** Turn the robot on */
    void turnOn();

    /** Turn the robot off */
    void turnOff();


private:

    ros::NodeHandle nodeH;
    std::string nameSpace;

    /** Waypoint navigation (Navigator) */
    std::shared_ptr<TaskReachedCallback> owner;

    State state;
    ros::Timer timerLoop;

    ros::Publisher pub_cmd;
    ros::Subscriber subRobotPose;

    geometry_msgs::Pose ownPos;
    geometry_msgs::Point newWpt;
    geometry_msgs::Twist velocity_cmd;

    // ------------  constants
    const double VELOCITY_MAX_MS = 0.8;
    const double TURN_MAX_RADS = 1.0;
    const double WPT_REACH_TOLERANCE_M = 0.2;
    const double WPT_HEADING_REACH_RAD = M_PI/8;

    const double DELTA = 5.0;
    const double R_MIN = 9.0;

    tf::Transform tf_w1, tf_w2, tf_carrot, tf_target;

    bool waypointReady = false;
    bool positionReady = false;


    /** Chenge current aircraft NAV state */
    void changeState(const State &newState);

    /** first reach heading without movement */
    void reachHeading();

    double getForwardSpeed(double Ru, double reqHeadingChange);

    double getAngularSpeed(double reqHeadingChange);

    double countRequiredHeadingChange() const;

    void turn(double requiredHeadingChange);
};

#endif //TECH_TASK_CONTROLLER_H
