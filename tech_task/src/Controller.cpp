#include "Controller.h"

Controller::Controller(ros::NodeHandle &nh, const std::string &nameSpace,const std::string positionSubscribeTopic,
                       const std::string velocityTopic, std::shared_ptr<TaskReachedCallback> owner) {
    this->nodeH = nh;
    this->nameSpace = nameSpace;
    this->owner = owner;
    changeState(State::OFF);


    pub_cmd = nh.advertise<geometry_msgs::Twist>(nameSpace + velocityTopic, 10);
    subRobotPose = nh.subscribe<geometry_msgs::Pose>(nameSpace + positionSubscribeTopic, 10, &Controller::ownStateCallback, this);

    // wpt navigation 10Hz
    timerLoop = nodeH.createTimer(ros::Duration(0.1), &Controller::checkStateLoop, this);
}


void Controller::ownStateCallback(const geometry_msgs::PoseConstPtr &msg) {
    ownPos.position = msg->position;
    ownPos.orientation = msg->orientation;
    positionReady = true;
}



void Controller::stop_robot() {
    changeState(State::STAND_BY);
    velocity_cmd = geometry_msgs::Twist();
}


double Controller::getForwardSpeed(double Ru, double reqHeadingChange) {

    double speed = VELOCITY_MAX_MS * cos(reqHeadingChange);
    speed = std::min(VELOCITY_MAX_MS,std::max(0.1, speed));
    return speed;
}

double Controller::getAngularSpeed(double reqHeadingChange) {
    if(reqHeadingChange > 0) {
        return std::min(TURN_MAX_RADS, reqHeadingChange);
    }else {
        return std::max(-TURN_MAX_RADS, reqHeadingChange);
    }
}


void Controller::move() {
    if (!isTargetReached()) {
        // Distance between robot and waypoint
        double Ru = utils::compute_euklidean_distance(ownPos.position, newWpt);
        double reqHeading = countRequiredHeadingChange();

        velocity_cmd.linear.x = getForwardSpeed(Ru, reqHeading);
        velocity_cmd.angular.z = getAngularSpeed(reqHeading);
    }
    else {
        stop_robot();
        owner->taskReached();
    }
}


double Controller::countRequiredHeadingChange() const {
    // Angle between connection of waypoint and robot
    double thetaW = atan2(newWpt.y - ownPos.position.y, newWpt.x - ownPos.position.x);

    // Direction angle of robot
    tf::Quaternion quaternion;
    utils::convert_to_tf_quaternion(ownPos.orientation, quaternion);
    double thetaR = getYaw(quaternion);

    double reqHeading = angles::shortest_angular_distance(thetaR, thetaW);
    return reqHeading;
}


bool Controller::isTargetReached() {
    double dist_to_target = utils::compute_euklidean_distance(ownPos.position, newWpt);
    return (dist_to_target < WPT_REACH_TOLERANCE_M);
}


void Controller::reachHeading() {
    // when position not yet acquired
    if(!positionReady){
        return;
    }
    double reqHeadingChange = countRequiredHeadingChange();
    if(std::abs(reqHeadingChange) > WPT_HEADING_REACH_RAD){
        turn(reqHeadingChange);
    }
    else {
        changeState(State::NAV_TO_WPT);
        move();
    }
}

void Controller::turn(double requiredHeadingChange) {
    velocity_cmd.linear.x = 0;
    velocity_cmd.angular.z = getAngularSpeed(requiredHeadingChange);
}

void Controller::publish_velocity() {
    pub_cmd.publish(velocity_cmd);
}


int Controller::setTargetWaypoint(const geometry_msgs::Point &point){
    if(state == State::OFF){
        ROS_ERROR("%s Sending waypoints when robot is OFF.", nameSpace.c_str());
        return -1;
    }
    newWpt = point;

    if (!waypointReady) {
        waypointReady = true;
    }
    stop_robot();   // stop robot, we do not need smooth movement when covering all the area
    changeState(State::REACHING_HEADING);

    return 0;
}



void Controller::turnOn() {
    if (state == State::OFF) {
        ROS_INFO("%s Turning on...", nameSpace.c_str());
        // give it some time to initialize
        ros::Duration(1.5).sleep();
        changeState(State::STAND_BY);
        timerLoop.start();
    }
}

void Controller::turnOff() {
    stop_robot();
    ROS_INFO("%s Turning off...", nameSpace.c_str());
    changeState(State::OFF);
    timerLoop.stop();
}

void Controller::changeState(const State &newState){
    if(state != newState) {
        state = newState;
//        ROS_INFO("Changed mode: %i", state);
    }
}


void Controller::checkStateLoop(const ros::TimerEvent &){
    switch (state) {
        case State::REACHING_HEADING:
            reachHeading();
            break;

        case State::NAV_TO_WPT:
            move();
            break;

        default:
            break;
    }
    publish_velocity();
}



