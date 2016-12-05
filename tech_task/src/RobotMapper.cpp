#include "RobotMapper.h"

RobotMapper::RobotMapper(ros::NodeHandle &nh, const std::string &nameSpace) {
    this->nodeH = nh;
    this->nameSpace = nameSpace;

    nh.param<double>("robot_radius", robotRadius, DEFAULT_ROBOT_RADIUS_M);
    nh.param<std::string>("pos_topic", positionSubscribeTopic, "/ground_truth/pose");
    nh.param<std::string>("vel_topic", velocityTopic, "/cmd_vel_mux/input/teleop");

    nh.param<double>("goal_x", goalX, 4.0);
    nh.param<double>("goal_y", goalY, 6.8);

    // subscribe map
    mapSubscriber = nodeH.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &RobotMapper::mapCallback, this);
    subRobotState = nodeH.subscribe<geometry_msgs::Pose>(nameSpace + positionSubscribeTopic, 10, &RobotMapper::ownStateCallback, this);

    initComponents();

    ros::spin();
}

void RobotMapper::initComponents() {

    communicator = std::shared_ptr<Communicator>(new Communicator(nodeH, nameSpace, positionSubscribeTopic, this));

    /* Sweep planner */
    sweepPlanner = std::shared_ptr<SweepingPlanner>(new SweepingPlanner(nodeH, nameSpace, positionSubscribeTopic, this));

    /* Path planner */
    navigator = std::shared_ptr<Navigator>(new Navigator(nodeH, nameSpace, positionSubscribeTopic, sweepPlanner));
    sweepPlanner->addNavigator(navigator);
    communicator->addNavigator(navigator);

    /* Motion controller */
    controller = std::shared_ptr<Controller>(new Controller(nodeH, nameSpace, positionSubscribeTopic, velocityTopic, navigator));
    navigator->addController(controller);

}


void RobotMapper::ownStateCallback(const geometry_msgs::PoseConstPtr &poseMsg){
    ownPose = poseMsg;
}


void RobotMapper::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &mapMsg) {
    ROS_INFO("Map arrived");
    map = std::shared_ptr<Map>(
            new Map((int) mapMsg->info.width, (int) mapMsg->info.height, (double) mapMsg->info.resolution,
                    mapMsg->data, mapMsg->info.origin.position));
    mapSubscriber.shutdown();

    sweepPlanner->setMap(map);
    navigator->setMap(map);

    boost::thread(boost::bind(&RobotMapper::startMission, this));
}


void RobotMapper::startMission() {
    controller->turnOn();
    estimateMasterAndStart();
}


void RobotMapper::estimateMasterAndStart() {
    // sleep thread to give time for faster voting
    boost::this_thread::sleep_for(boost::chrono::seconds(6));

    if(communicator->amIMaster()){
        ROS_INFO("%s is a MASTER", nameSpace.c_str());
        amIMaster = true;
        assignSweepArea();
    }
}


void RobotMapper::passMissionToPlanner(Area area, Position goal){
    sweepPlanner->sweepArea(area, goal);
}


void RobotMapper::missionArrived() {
    // reach to communicator for arrived mission
    std::vector<Position> mission;
    communicator->getMissionData(mission);
    if(mission.size() > 1) {
        Area area = Area(mission[0], mission[1]);
        Position goal(goalX, goalY);

        // start mission in new thread to free subscriber callback
        boost::thread(boost::bind(&RobotMapper::passMissionToPlanner, this, area, goal));
    }
    else{
        ROS_WARN("%s incomplete mission arrived.", nameSpace.c_str());
    }
}

void RobotMapper::assignSweepArea() {
    // get neigbor positions to assign the subscribed tasks
    std::unordered_map<std::string, Position> ngbrPoses;
    communicator->getNgbrPoses(ngbrPoses);
    Position myPos(ownPose->position.x, ownPose->position.y);
    ngbrPoses.emplace(nameSpace, myPos);

    int numOfRobots = (int)ngbrPoses.size();


    // get whole area corners
    std::vector<Position> corners;
    map->getMapFreeCornersInM(corners, robotRadius);
    if(corners.size() != 2){
        ROS_WARN("Incorrect number of corners found.");
    }

    std::vector<Area> areas;
    sweepPlanner->divideArea(corners, numOfRobots, areas);

    std::unordered_map<std::string, Area> missions;
    assignAreasToRobots(areas, ngbrPoses, missions);

    Position start = missions[nameSpace].start;
    Position end = missions[nameSpace].end;
    Area myArea(start, end);

    auto it = missions.find(nameSpace);
    if(it != missions.end()){
        missions.erase(it);
    }

    // distribute mission tasks
    communicator->distributeMission(missions);

    // plan mission
    Position goal(goalX, goalY);
    sweepPlanner->sweepArea(myArea, goal);
}



void RobotMapper::assignAreasToRobots(std::vector<Area> areas, std::unordered_map<std::string, Position> ngbrPoses,
                                      std::unordered_map<std::string, Area> &mission) {

    std::vector<std::string> assignedRobots;
    for(auto area : areas){
        std::string closest;
        findClosestRemaining(ngbrPoses, assignedRobots, area.start, closest);
        mission[closest] = area;
        assignedRobots.push_back(closest);
    }
}


void RobotMapper::findClosestRemaining(const std::unordered_map<std::string, Position> &ngbrPoses,
                                       const std::vector<std::string> &alreadyAssigned,
                                       Position areaStart, std::string &closestRobot) {
    double minDist = INT_MAX;
    std::string closest;
    for(auto pair : ngbrPoses){
        if(std::find(alreadyAssigned.begin(), alreadyAssigned.end(), pair.first) == alreadyAssigned.end()){
            Position p = pair.second;
            double dist = utils::compute_euklidean_distance(p.x, p.y, areaStart.x, areaStart.y);
            if(dist < minDist){
                minDist = dist;
                closest = pair.first;
            }
        }

    }
    closestRobot = closest;
}


void RobotMapper::taskReached() {
    communicator->informMissionFinished();
    // turn off robot
    controller->turnOff();
}


int main(int argc, char *argv[]){
    std::cout<<"STARTED SWEEP PLANNER"<<std::endl;

    ros::init(argc, argv, "sweep_planner");
    ros::NodeHandle nh;

    ros::Duration(2.0).sleep();

    RobotMapper sweepPlanner(nh, ros::this_node::getNamespace());  // +"/robot1"

    return 0;
}