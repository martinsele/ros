#include "Communicator.h"


Communicator::Communicator(ros::NodeHandle &nh, const std::string &nameSpace,
                           const std::string positionSubscribeTopic, CommunicatorCallback* owner):
        goal(Position(0,0)),
        area(Area(goal, goal)),
        positionSubscribeTopic(positionSubscribeTopic){

    this->nodeH = nh;
    this->nameSpace = nameSpace;
    this->owner = owner;

    Position pos(Position(0,0));
    area = Area(pos, pos);

    subRobotPose = nodeH.subscribe<geometry_msgs::Pose>(nameSpace + positionSubscribeTopic, 10,
                                                        &Communicator::ownStateCallback, this);
    subOtherNames = nodeH.subscribe("/own_name", 10, &Communicator::processNeighborsCallback, this); // subscribe other neighbors names
    subMission = nodeH.subscribe("/mission", 10, &Communicator::missionArrived, this);  // subscribe for mission from master

    pubOwnName = nodeH.advertise<std_msgs::String>("/own_name",10); // advertise own name topic
    pubPlan = nodeH.advertise<nav_msgs::Path>(nameSpace + PLAN_SUFFIX, 10); // publishing plans to subscribed lower priority neighbors
    pubMission = nodeH.advertise<nav_msgs::Path>("/mission", 10);   // advertise mission topic for case of being master

    timerName = nodeH.createTimer(ros::Duration(2), &Communicator::informNameLoop, this);    // inform own name 0.5Hz
    timerPlan = nodeH.createTimer(ros::Duration(2), &Communicator::broadcastPlanLoop, this); // broadcast current plan 0.5Hz

}


/****************************************************************************/
/**             HANDLE TOPIC CALLBACKS                                      */
/****************************************************************************/

void Communicator::ownStateCallback(const geometry_msgs::PoseConstPtr &msg) {
    ownPose.position = msg->position;
    ownPose.orientation = msg->orientation;
    ownPoseArrived = true;
}

void Communicator::processNeighborsCallback(const std_msgs::String::ConstPtr &nbrName) {
    // process only foreign messages
    std::string name = nbrName->data;

    // if not myself and new, save and subscribe
    if(name.compare(nameSpace) && neighborNames.find(name) == neighborNames.end()){
        handleNewNeighbor(name);
    }
}

void Communicator::processNeighborState(const geometry_msgs::PoseConstPtr &msg, std::string &neighborName) {
    Position p(msg->position.x, msg->position.y);
    neighborPoses[neighborName] = p;

    // inform navigator
    navigator->updateNbrPosition(neighborName, p);
}


void Communicator::processNgbrPlan(const nav_msgs::Path::ConstPtr &msg) {
    // do not process own topics
    if(!msg->header.frame_id.compare(nameSpace)){
        return;
    }
    // handle arrived messages with plans
    std::string ngbrName = msg->header.frame_id;
    Position ngbrNextWpt;
    for(auto wpt : msg->poses){
        ngbrNextWpt = Position(wpt.pose.position.x, wpt.pose.position.y);
    }
    neighborPlans[ngbrName] = ngbrNextWpt;

    // inform navigator
    navigator->updateNbrWpts(ngbrName, ngbrNextWpt);
}



void Communicator::missionArrived(const nav_msgs::PathConstPtr &mission) {
    // standard message
    if(mission->header.frame_id.empty()) {
        if (master) {
            return;
        }
        // find my corners
        std::vector<Position> corners;
        for (auto point : mission->poses) {
            if (!point.header.frame_id.compare(nameSpace)) {
                Position p(point.pose.position.x, point.pose.position.y);
                corners.push_back(p);
            }
        }
        if (corners.size() > 1) {
            area = Area(corners[0], corners[1]);
            // send info about arrived mission to RobotMapper
            owner->missionArrived();
        } else {
            ROS_WARN("%s incomplete mission arrived.", nameSpace.c_str());
        }
    }
    // mission finished message
    else {
        std::string finishedRobot = mission->header.frame_id;
        Position robotsFinalPose;
        for (auto point : mission->poses) {
            robotsFinalPose = Position(point.pose.position.x, point.pose.position.y);
        }
        ROS_INFO("neighbor finished: %s", finishedRobot.c_str());
        navigator->neighborFinished(finishedRobot, robotsFinalPose);
    }
}


/****************************************************************************/
/**             NEIGHBOR HANDLING                                           */
/****************************************************************************/

void Communicator::handleNewNeighbor(std::string &name) {
    neighborNames.insert(name);
    subscribeNbrPos(name);
    // advertise topic to higher priority entity (priorities based on agent name)
    if(name.compare(nameSpace) > 0) {
        insertHighPriorityNgbr(name);
        master = false;
    }
}

void Communicator::subscribeNbrPos(std::string &neighborName) {
    ROS_WARN("%s subscribe neighbor: %s", nameSpace.c_str(), neighborName.c_str());

    // subscribe for particular neighbor's position
    ros::Subscriber sub = nodeH.subscribe<geometry_msgs::Pose>(
            neighborName+positionSubscribeTopic, 20,
            boost::bind(&Communicator::processNeighborState, this, _1, neighborName));

    // remember the subscriber
    otherStatesSubscribers.push_back(sub);
}


void Communicator::insertHighPriorityNgbr(std::string ngbrName){
    if(higherPriorityNghbrs.find(ngbrName) == higherPriorityNghbrs.end()){
        higherPriorityNghbrs.insert(ngbrName);

        ROS_INFO("%s: subscribe plan from %s", nameSpace.c_str(), ngbrName.c_str());

        // subscribe for plan of higher priority entities
        ros::Subscriber pub = nodeH.subscribe(ngbrName + PLAN_SUFFIX, 10, &Communicator::processNgbrPlan, this);
        plansSubscribers.push_back(pub);
    }
}


void Communicator::getNgbrPoses(std::unordered_map<std::string, Position> &retPoses) {
    for(auto pair : neighborPoses){
        retPoses[pair.first] = pair.second;
    }
}


/****************************************************************************/
/**             HANDLE TIMERS LOOPS                                         */
/****************************************************************************/


void Communicator::informNameLoop(const ros::TimerEvent &){
    std_msgs::String name_msg;
    name_msg.data = nameSpace;
    // publish agent's name
    pubOwnName.publish(name_msg);
}



void Communicator::broadcastPlanLoop(const ros::TimerEvent &) {
    // broadcast own plan to high priority ngbrs
    Position wpt;
    navigator->getCurrentWaypoint(wpt);

    nav_msgs::Path path;
    path.header.frame_id = nameSpace;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = wpt.x;
    pose.pose.position.y = wpt.y;
    pose.header.stamp = ros::Time();

    path.poses.push_back(pose);

    pubPlan.publish(path);

}

/****************************************************************************/
/**             OTHER                                                       */
/****************************************************************************/

void Communicator::addNavigator(std::shared_ptr<Navigator> navigator) {
    this->navigator = navigator;
}

bool Communicator::amIMaster() {
    return master;
}


void Communicator::informMissionFinished(){
    // send special mission message - fith header name corresponding to my name and one point (my position)
    nav_msgs::Path path;
    path.header.frame_id = nameSpace;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = ownPose.position.x;
    pose.pose.position.y = ownPose.position.y;
    path.poses.push_back(pose);
    pubMission.publish(path);
}

void Communicator::distributeMission(const std::unordered_map<std::string, Area> &missions) {
    // create Path message and distribute it via topic
    nav_msgs::Path path;
    for(auto robot : missions){
        // for each robot add 2 points - area start, area end
        geometry_msgs::PoseStamped posStart;
        posStart.header.frame_id = robot.first;  // add robot's name
        posStart.pose.position.x = robot.second.start.x;
        posStart.pose.position.y = robot.second.start.y;
        path.poses.push_back(posStart);

        geometry_msgs::PoseStamped posEnd;
        posEnd.header.frame_id = robot.first;  // add robot's name
        posEnd.pose.position.x = robot.second.end.x;
        posEnd.pose.position.y = robot.second.end.y;
        path.poses.push_back(posEnd);

    }
    if(missions.size() > 0) {
        pubMission.publish(path);
    }
}

void Communicator::getMissionData(std::vector<Position> &mission) {
    mission.push_back(area.start);
    mission.push_back(area.end);
}





