#include "Navigator.h"


/**
 * Define priority queue structure for A* OPEN list
 */
struct PriorityQueue {
    std::priority_queue<std::pair<double, GraphLocation>,
            std::vector<std::pair<double, GraphLocation>>,
            std::greater<std::pair<double, GraphLocation>>> elements;

    bool empty() const { return elements.empty(); }

    void put(GraphLocation item, double priority) {
        elements.emplace(priority, item);
    }

    GraphLocation get() {
        GraphLocation best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};


Navigator::Navigator(ros::NodeHandle &nh, const std::string &nameSpace, const std::string positionSubscribeTopic,
                     std::shared_ptr<TaskReachedCallback> owner) : currentMissionWpt(-100,-100){
    this->nodeH = nh;
    this->nameSpace = nameSpace;
    this->owner = owner;

    nh.param<double>("robot_radius", robotRadius, DEFAULT_ROBOT_RADIUS_M);

    changeState(NavState::WAITING_FOR_DATA);

    // subscribe robot position
    subRobotState = nodeH.subscribe<geometry_msgs::Pose>(nameSpace + positionSubscribeTopic, 10, &Navigator::ownStateCallback, this);

    // navigation timer 10Hz
    timerLoop = nodeH.createTimer(ros::Duration(0.1), &Navigator::checkStateLoop, this);
}


void Navigator::addController(std::shared_ptr<Controller> controller) {
    this-> controller = controller;
}


/****************************************************************************/
/**             DATA UPDATES                                                */
/****************************************************************************/


void Navigator::ownStateCallback(const geometry_msgs::PoseConstPtr &poseMsg) {
    ownPose.position = poseMsg->position;
    ownPose.orientation = poseMsg->orientation;
    ownPoseSet = true;
}

void Navigator::updateNbrPosition(std::string nbrName, const Position &pose) {
    boost::unique_lock<boost::shared_mutex> lock(mutex_pos_);

    // update neighbor's position
    Position p(pose.x, pose.y);
    neighborPoses[nbrName] = p;
}

void Navigator::updateNbrWpts(std::string nbrName, const Position &wpt) {
    boost::unique_lock<boost::shared_mutex> lock(mutex_plans_);

    // update neighbor names if necessary
    nghbrNames.insert(nbrName);

    // update neighbor's next waypoint
    Position p(wpt.x, wpt.y);
    neighborPlans[nbrName] = p;
}



/****************************************************************************/
/**             PLAN FOLLOWING                                              */
/****************************************************************************/


void Navigator::taskReached() {
//    ROS_INFO("%s arrived at loc wpt", nameSpace.c_str());
    if(state != AVOIDING) {
        wptQueue.pop_front();

        // upload next control waypoint
        if (!wptQueue.empty()) {
            uploadWptToController(wptQueue.front());
        }
            // when all wpts reached, inform owner and ask for next mission wpt
        else {
            currWptSet = false;
            changeState(NavState::WAITING_FOR_DATA);
            owner->taskReached();
        }
    }
    else {
        // no need to process avoiding waypoints
    }
}

/**
 * Find collision-free path to current mission waypoint
 */
void Navigator::plan() {
    if(!currWptSet || !mapSet || !ownPoseSet ){
        return;
    }

    changeState(NavState::NAVIGATING);

    GraphLocation start(ownPose.position.x, ownPose.position.y);
    GraphLocation goal(currentMissionWpt.x, currentMissionWpt.y);
    std::unordered_map<GraphLocation, GraphLocation> came_from;
    std::unordered_map<GraphLocation, double> cost_so_far;

    bool found = aStarSearch(graph, start, goal, came_from, cost_so_far);
    if(!found){
        ROS_WARN("%s Path not found (%f,%f)->(%f,%f)",nameSpace.c_str(), start.getX(),start.getY(), goal.getX(), goal.getY());
    }

    reconstructPath(start,goal, came_from, wptQueue);
    printPath(wptQueue);

    uploadWptToController(wptQueue.front());
}

void Navigator::planPathTo(Position &missionWpt) {
//    ROS_INFO("%s new mission wpt (%f,%f)", nameSpace.c_str(), missionWpt.x, missionWpt.y);
    currentMissionWpt = missionWpt;
    currWptSet = true;
}


void Navigator::changeState(const NavState &newState){
    if(state != newState) {
        state = newState;
    }
}


void Navigator::checkStateLoop(const ros::TimerEvent &){
    switch (state) {
        case NavState::WAITING_FOR_DATA:
            plan();
            break;

        case NavState::NAVIGATING:
            checkExecution();
            break;

        case NavState ::AVOIDING:
            checkAvoidance();
            break;
        default:
            break;
    }
}

void Navigator::uploadWptToController(GraphLocation &location) {
    geometry_msgs::Point point;
    point.x = location.getX();
    point.y = location.getY();
    controller->setTargetWaypoint(point);
}

void Navigator::setMap(std::shared_ptr<Map> &map) {
    this->map = map;
    graph = std::shared_ptr<SearchGraph>(new SearchGraph(map, robotRadius));
    mapSet = true;
}

double Navigator::getRobotRadius() {
    return robotRadius;
}

void Navigator::getCurrentWaypoint(Position &curWpt) {
    curWpt.x = currentMissionWpt.x;
    curWpt.y = currentMissionWpt.y;
}


/****************************************************************************/
/**             NEIGHBOR AVOIDING                                           */
/****************************************************************************/

void Navigator::checkExecution() {
    // monitor execution (colllision between current wpt and position?), if possible collision, stop robot and replan
    Position curPos(ownPose.position.x, ownPose.position.y);
    std::vector<std::string> nearNbrs;
    checkNearNgbrs(curPos, nearNbrs);

    if(!nearNbrs.empty()){
        ROS_WARN("%s too near, avoiding:", nameSpace.c_str());
        // safe current position to the begining of waypoint list
        changeState(NavState::AVOIDING);
        GraphLocation curentPos(ownPose.position.x, ownPose.position.y);
        wptQueue.push_front(curentPos);

        // handle avoidance
        handleNearNgbrs(curPos, nearNbrs);
    }
}


void Navigator::checkNearNgbrs(const Position &currentPos, std::vector<std::string> &nearNbrs) {
    boost::shared_lock<boost::shared_mutex> lock(mutex_pos_);

    for(auto ngbrPos : neighborPoses){
        // skip myself, lower priority robots without plan and finished robots, those are handled somewhere else
        if(!ngbrPos.first.compare(nameSpace) ||
                neighborPlans.find(ngbrPos.first) == finishedNeighbors.end() ||
                finishedNeighbors.find(ngbrPos.first) != finishedNeighbors.end()){
            continue;
        }
        double dist = utils::compute_euklidean_distance(currentPos.x, currentPos.y, ngbrPos.second.x, ngbrPos.second.y);

        if(dist < 3* robotRadius){
            nearNbrs.push_back(ngbrPos.first);
        }
    }
}


void Navigator::checkAvoidance() {
    Position curPos(ownPose.position.x, ownPose.position.y);
    std::vector<std::string> nearNbrs;
    checkNearNgbrs(curPos, nearNbrs);

    if(!nearNbrs.empty()){
        handleNearNgbrs(curPos, nearNbrs);
    }
    else {
        returnToPlan();
    }
}


void Navigator::handleNearNgbrs(const Position curPos, const std::vector<std::string> &nearNgbrs) {
    std::vector<Position> nearObstacles;
    map->getClosestObstaclePoints(curPos, 3*robotRadius, robotRadius, nearObstacles);
    Position dirToPush;
    countAvoidancePushDirection(curPos, nearNgbrs, nearObstacles, dirToPush);

    GraphLocation avoidWpt(2*dirToPush.x*robotRadius, 2*dirToPush.y*robotRadius);

    // upload evasion waypoint
    uploadWptToController(avoidWpt);
}


void Navigator::countAvoidancePushDirection(const Position curPosition, const std::vector<std::string> &nearNeighbors,
                                            const std::vector<Position> &nearObstacles, Position &push) {
    double dirX = 0;
    double dirY = 0;

    for (auto obstacle : nearObstacles) {
        dirX += 0.2 * getPushAmplitude(curPosition.x, obstacle.x);  // push a bit further from obstacle
        dirY += 0.2 * getPushAmplitude(curPosition.y, obstacle.y);
    }

    // lock access to neighbors
    boost::shared_lock<boost::shared_mutex> lock(mutex_plans_);

    for(auto ngbr : nearNeighbors){
        Position ngbrPos = neighborPoses.at(ngbr);
        Position ngbrWpt = neighborPlans.at(ngbr);

        dirX += 3 * getPushAmplitude(curPosition.x, ngbrPos.x);  // push further from robot
        dirY += 3 * getPushAmplitude(curPosition.y, ngbrPos.y);

        dirX += 2 * getPushAmplitude(curPosition.x, ngbrWpt.x);  // push less further from its trajectory
        dirY += 2 * getPushAmplitude (curPosition.y, ngbrWpt.y);
    }

    push.x = dirX;
    push.y = dirY;

}

double Navigator::getPushAmplitude(double robotCoord, double obstacleCoord){
    int dirPos = (robotCoord > obstacleCoord) ? 1 : -1;

    double den = std::max(0.1, std::abs(robotCoord - obstacleCoord));
    double ampl = dirPos / den;
    return ampl;
}

void Navigator::returnToPlan() {
    ROS_WARN("%s Returning to plan..", nameSpace.c_str());
    uploadWptToController(wptQueue.front());
    changeState(NAVIGATING);
}

void Navigator::neighborFinished(const std::string finishedRobot, const Position robotsFinalPose) {
    finishedNeighbors[finishedRobot] = robotsFinalPose;
    map->otherRobotFinished(finishedRobot, robotsFinalPose);
}

/****************************************************************************/
/**             A* PLANNING                                                 */
/****************************************************************************/


bool Navigator::aStarSearch (std::shared_ptr<SearchGraph> graph, GraphLocation start, GraphLocation &goal,
                             std::unordered_map<GraphLocation, GraphLocation>& came_from,
                             std::unordered_map<GraphLocation, double>& cost_so_far)
{
    // check goal in free area
    bool mapFree = !map->isOccupiedRad(goal.getX(), goal.getY(), robotRadius);
    if(!mapFree){
        // move goal closer to my start
        Position g(goal.getX(), goal.getY());
        Position s(start.getX(), start.getY());
        Position newGoal;
        map->getFirstFreePos(g, s, newGoal, robotRadius);
        ROS_INFO("%s Goal (%f,%f) not free, moved to (%f,%f)", nameSpace.c_str(), goal.getX(), goal.getY(), newGoal.x, newGoal.y);
        goal = GraphLocation (newGoal.x, newGoal.y);
    }

    bool found = false;
    PriorityQueue frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        auto current = frontier.get();

        if (current.isSame(goal)) {
            found = true;
            break;
        }

        std::vector<GraphLocation> nextNodes = graph->neighbors(current, goal);
        for (auto next : nextNodes) {
            double new_cost = cost_so_far[current] + graph->getCost(current, next);
            if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
    return found;
}

double Navigator::heuristic(GraphLocation loc, GraphLocation goal) {
    return utils::compute_euklidean_distance(loc.getX(), loc.getY(), goal.getX(), goal.getY());;
}



void Navigator::reconstructPath(GraphLocation start, GraphLocation goal,
                                std::unordered_map<GraphLocation, GraphLocation> &cameFrom,
                                std::deque<GraphLocation> &path) {
    path.clear();
    GraphLocation current = goal;
    while (!current.isSame(start)) {
        path.push_front(current);
        if(cameFrom.find(current) == cameFrom.end()){
            ROS_WARN("%s could not reconstruct plan for node (%f, %f)", nameSpace.c_str(), current.getX(), current.getY());
            break;
        }
        current = cameFrom[current];
    }
}


void Navigator::printPath(std::deque<GraphLocation> path) {
    std::ostringstream out;
    for (std::deque<GraphLocation>::iterator node = path.begin(); node!=path.end(); ++node){
        out << "-> (" << node->getX() << ", " << node->getY() << ")";
    }
    ROS_INFO("%s path: %s", nameSpace.c_str(), out.str().c_str());
}







