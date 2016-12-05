#include "SweepingPlanner.h"

SweepingPlanner::SweepingPlanner(ros::NodeHandle &nh, const std::string &nameSpace,
                                 const std::string positionSubscribeTopic, TaskReachedCallback *owner):
        goal(Position(0,0)), areaToSweep(Area(goal, goal)) {

    this->nodeH = nh;
    this->nameSpace = nameSpace;
    this->owner = owner;

    subRobotState = nodeH.subscribe<geometry_msgs::Pose>(nameSpace + positionSubscribeTopic, 10, &SweepingPlanner::ownStateCallback, this);
    // check arrived data
    dataCheckLoop = nodeH.createTimer(ros::Duration(0.1), &SweepingPlanner::checkStateLoop, this);
    // visualize
    marker_pub = nodeH.advertise<visualization_msgs::Marker>(nameSpace+"/visualization_marker", 10);
}



void SweepingPlanner::taskReached() {
    ROS_INFO("%s Reached Mission waypoint", nameSpace.c_str());

    if(areaSet) {
        remainingMission.erase(remainingMission.begin());

        // upload next mission waypoint
        if (!remainingMission.empty()) {
            navigator->planPathTo(remainingMission[0]);
        }
            // when all wpts reached, inform owner and go to goal position
        else {
            navigator->planPathTo(goal);
            areaSet = false;
        }
    }
    else{
        // reached goal position
        ROS_INFO("%s Mission finished", nameSpace.c_str());
        owner->taskReached();
    }
}

void SweepingPlanner::ownStateCallback(const geometry_msgs::PoseConstPtr &poseMsg) {
    ownPose.position = poseMsg->position;
    ownPose.orientation = poseMsg->orientation;
    ownPoseArrived = true;

    // visualize path
    if(posCounter > 200){
        posCounter = 0;
        Position pos(ownPose.position.x, ownPose.position.y);
        path.push_back(pos);
        visualizePath(pos);
    }
    posCounter++;
}

void SweepingPlanner::addNavigator(std::shared_ptr<Navigator> navigator) {
    this->navigator = navigator;
}

void SweepingPlanner::sweepArea(const Area &area, const Position &goalPosition) {
    areaToSweep = area;
    areaSet = true;
    goal = goalPosition;
    planning = false;

    ROS_INFO("%s sweepoing area (%f,%f)-(%f,%f) with goal (%f,%f)",
             nameSpace.c_str(), area.start.x, area.start.y, area.end.x, area.end.y, goal.x, goal.y);

    if(mapArrived && ownPoseArrived){
        planning = true;
        // stop checking loop
        dataCheckLoop.stop();

        std::vector<Position> wptPattern;
        createSweepPattern(wptPattern);

        visualizePattern(wptPattern);
        remainingMission = wptPattern;

        if(remainingMission.size() > 0) {
            navigator->planPathTo(remainingMission[0]);
        }
        else{
            ROS_WARN("%s SweepingPlanner could not generate sweep pattern", nameSpace.c_str());
        }
    }
}

void SweepingPlanner::createSweepPattern(std::vector<Position> &waypoints) {
    // get sweep pattern direction
    std::vector<Position> vec;
    createCornerVector(areaToSweep, vec);
    visualizeArea(vec);

    Position closest;
    Position mostDist;
    findCloseDist(closest, mostDist, vec);

    double horiz = utils::compute_euklidean_distance(vec[0].x, vec[0].y, vec[3].x, vec[3].y);
    double vert = utils::compute_euklidean_distance(vec[3].x, vec[3].y, vec[2].x, vec[2].y);

    bool useHorizontal = horiz/vert >= 1;

    // get waypoints
    getSweepPattern(closest, mostDist, useHorizontal, waypoints);
}


void SweepingPlanner::createCornerVector(Area area, std::vector<Position> &vec){
    // leftDown
    vec.push_back(Position(std::min(area.start.x, area.end.x),
                           std::min(area.start.y, area.end.y)));
    // leftUp
    vec.push_back(Position(std::min(area.start.x, area.end.x),
                           std::max(area.start.y, area.end.y)));
    // rightUp
    vec.push_back(Position(std::max(area.start.x, area.end.x),
                           std::max(area.start.y, area.end.y)));
    // rightDown
    vec.push_back(Position(std::max(area.start.x, area.end.x),
                           std::min(area.start.y, area.end.y)));
}


void SweepingPlanner::getSweepPattern(const Position &closestP, const Position &mostDistP, bool useHorizontal,
                                      std::vector<Position> &waypoints) const {
    // define sweep patterns
    int patternShort[] = {0, 1};
    int patternLong[] = {1, 0, -1, 0};
    Pattern HORIZONTAL(patternLong, patternShort);
    Pattern VERTICAL(patternShort, patternLong);

    Pattern pattern = (useHorizontal) ? HORIZONTAL : VERTICAL;
    int patternX_len = (useHorizontal) ? 4 : 2;
    int patternY_len = (useHorizontal) ? 2 : 4;

    // directions of sweeping
    double distanceX = std::abs(mostDistP.x - closestP.x);
    double distanceY = std::abs(mostDistP.y - closestP.y);
    double dirX = (mostDistP.x - closestP.x)/distanceX;
    double dirY = (mostDistP.y - closestP.y)/distanceY;

    double turnLength = 1.8 * navigator->getRobotRadius(); // have some path overlay

    int xTurns, yTurns;
    double remain; // if one more turn is needed
    if(useHorizontal){
        xTurns = 1;
        yTurns = (int) (distanceY / turnLength);
    }else{
        xTurns = (int) (distanceX / turnLength);
        yTurns = 1;
    }
    // get lengths of segments between waypoints
    double turnLenX = distanceX / xTurns;
    double turnLenY = distanceY / yTurns;


    waypoints.push_back(closestP);
    int iterNum = xTurns * yTurns * 2 + 1;
    int i = 0;
    for(i; i < iterNum; ++i){
        // count increments
        double dx = dirX*turnLenX * pattern.x[i % patternX_len];
        double dy = dirY*turnLenY * pattern.y[i % patternY_len];

        Position prev(waypoints.back().x, waypoints.back().y);
        Position p(prev.x + dx, prev.y + dy);

        /** insert waypoints at obstacles' edges if necessary*/
        map->getObstacleEdgesAtLine(prev, p, waypoints, navigator->getRobotRadius());

        // push waypoint
        waypoints.push_back(p);
    }
}

void SweepingPlanner::findCloseDist(Position &near, Position &far, const std::vector<Position> & corners) {

    int minDist = INT_MAX;
    int maxDist = -INT_MAX;
    int minIdx = 0;
    int maxIdx = 0;

    int i = 0;
    for(auto c : corners){
        int dist = (int)utils::compute_euklidean_distance(ownPose.position.x, ownPose.position.y, c.x, c.y);
        if(dist < minDist){
            minDist = dist;
            minIdx = i;
        }
        if(dist > maxDist){
            maxDist = dist;
            maxIdx = i;
        }
        i++;
    }
    near = corners[minIdx];
    far = corners[maxIdx];
}



void SweepingPlanner::setMap(std::shared_ptr<Map> &map) {
    this->map = map;
    mapArrived = true;
}


void SweepingPlanner::checkStateLoop(const ros::TimerEvent &){
    if(mapArrived && ownPoseArrived && !planning && areaSet){
        sweepArea(areaToSweep, goal);
    }
}

void SweepingPlanner::divideArea(std::vector<Position> corners, int parts, std::vector<Area> &areas){
    Area a(corners[0], corners[1]);
    std::vector<Position> vec;
    createCornerVector(a, vec);

    double horiz = utils::compute_euklidean_distance(vec[0].x, vec[0].y, vec[3].x, vec[3].y);
    double vert = utils::compute_euklidean_distance(vec[3].x, vec[3].y, vec[2].x, vec[2].y);
    bool useHorizontal = horiz/vert >= 1;

    double divLength = (useHorizontal) ? vert/parts : horiz/parts;

    for(int i = 0; i < parts; i++){
        if(useHorizontal){  // 0, 2
            Position ld (vec[0].x, vec[0].y + i*divLength);     // left down corner
            Position ru (vec[2].x, vec[0].y + (i+1)*divLength); // righ up corner
            Area area(ld,ru);
            areas.push_back(area);
        }
        else{  // 0, 2
            Position ld (vec[0].x + i*divLength, vec[0].y);     // left down corner
            Position ru (vec[0].x + (i+1)*divLength, vec[2].y); // righ up corner
            Area area(ld,ru);
            areas.push_back(area);
        }
    }
}


/****************************************************************************/
/**             VISUALIZATION                                               */
/****************************************************************************/

/**
 * Visualization of sweep waypoints
 * @param waypoints
 */
void SweepingPlanner::visualizePattern(std::vector<Position> &waypoints){
    visualization_msgs::Marker points;

    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Create the vertices for the points and lines
    for (int i = 0; i < waypoints.size(); i++) {
        float x = waypoints[i].x;
        float y = waypoints[i].y;

        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.1;
        points.points.push_back(p);

    }
    marker_pub.publish(points);

}

void SweepingPlanner::visualizeArea(std::vector<Position> vector) {
    visualization_msgs::Marker line_list;

    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 1;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // POINTS markers use x and y scale for width/height respectively
    line_list.scale.x = 0.05;
    line_list.scale.y = 0.05;
    // Points are red
    line_list.color.r = 1.0f;
    line_list.color.a = 1.0;

    // Create the vertices for the points and lines
    for (int i = 0; i < 4; i++) {
        Position v1 = vector[i];
        Position v2 = vector[(i+1) % 4];

        geometry_msgs::Point p1;
        p1.x = v1.x;
        p1.y = v1.y;
        p1.z = 0.1;
        geometry_msgs::Point p2;
        p2.x = v2.x;
        p2.y = v2.y;
        p2.z = 0.1;
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }
    marker_pub.publish(line_list);
}

void SweepingPlanner::visualizePath(const Position &pos) {
    visualization_msgs::Marker points;

    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 2;
    points.type = visualization_msgs::Marker::LINE_STRIP;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 1.0;
    points.scale.y = 1.0;
    // Points are green
    points.color.b = 1.0f;
    points.color.a = 0.4;

    // Create the vertices for the points and lines
    for (int i = 0; i < path.size(); i++) {
        double x = path[i].x;
        double y = path[i].y;

        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;
        points.points.push_back(p);

    }
    marker_pub.publish(points);
};
