#include "Map.h"

Map::Map(const int width, const int height, const double resolution, const std::vector<int8_t> &data, const geometry_msgs::Point &mapOrigin) {
    this->width = width;
    this->height = height;
    this-> resolution = round(resolution*100.0)/100.0;
    this->map = data;
    origX = mapOrigin.x;
    origY = mapOrigin.y;

    Position lastPosition;
    idxToM(width*height-1, lastPosition);
    this->lastPosition = lastPosition;

}

bool Map::isOccupied(double x, double y) {
    int index = getIndexToMap(x,y);
    if (index >= 0){
        if (map[index] > 0){
            return true;
        }
        return false;
    }
    // outside of map is occupied
    return true;
}


bool Map::isOccupiedRad(double x, double y, double radius){
    int dx, dy;
    // check the point itself
    bool occ = isOccupied(x,y);
    if(occ) {
        return true;
    }
    // check neighborhood in all directions
    for(auto dir : DIRS){
        dx = dir[0];
        dy = dir[1];
        // check whole line without radius
        occ = lineColision(x, y, (x + dx * radius), (y + dy * radius), -1);
        if(occ){
            return true;
        }
    }

    // check finished neighbors

    // add checking of neighbours
    for(auto robot : finishedRobots){
        double dist = utils::compute_euklidean_distance(x,y, robot.second.x, robot.second.y);
        if(dist < 2*radius){
            return true;
        }
    }

    return false;
}


bool Map::lineColision(double x1, double y1, double x2, double y2, double radius) {
    bool occupied;
    double dx, dy;
    int iterations;
    getLineIncrements(x1, y1, x2, y2, dx, dy, iterations);

    // check points from point (x1, y1) to (x2, y2)
    for(int i = 0; i < iterations; i++){
        x1 += dx;
        y1 += dy;
        occupied = (radius > 0) ? isOccupiedRad(x1, y1, radius) : isOccupied(x1, y1);
        if(occupied){
            return true;
        }
    }

    return false;
}

void Map::getLineIncrements(double x1, double y1, double x2, double y2, double &dx, double &dy, int &iterations) const{
    double dir = std::atan2((y2-y1), (x2-x1));
    double dist = utils::compute_euklidean_distance(x1, y1, x2, y2);

    // number of iterations needed to check all points
    iterations = (int)std::ceil(dist/this->resolution);

    // increments in both directions
    dx = this->resolution * cos(dir);
    dy = this->resolution * sin(dir);
}


void Map::getObstacleEdgesAtLine(const Position &first, const Position &last, std::vector<Position> &foundEdges, double radius){
    if(!lineColision(first.x, first.y, last.x, last.y, radius)){
        return;
    }
    if(!isOccupiedRad(first.x, first.y, radius)){
        Position foundLastFree;
        Position obstacleAt;
        if(getLastNonOccupPos(first, last, foundLastFree, obstacleAt, radius)){
            foundEdges.push_back(foundLastFree);
            getObstacleEdgesAtLine(obstacleAt, last, foundEdges, radius);
        }
    }
    else {
        Position foundFree;
        if(getFirstFreePos(first, last, foundFree, radius)){
            foundEdges.push_back(foundFree);
            getObstacleEdgesAtLine(foundFree, last, foundEdges, radius);
        }
    }
}


bool Map::getFirstFreePos(const Position &obstEdge, const Position &finalSearchPoint, Position &foundFree, double radius){
    double dx, dy;
    int iterations;
    double x1 = obstEdge.x;
    double y1 = obstEdge.y;
    getLineIncrements(obstEdge.x, obstEdge.y, finalSearchPoint.x, finalSearchPoint.y, dx, dy, iterations);

    // check points from point obstEdge to finalSearchPoint (excluded)
    for(int i = 0; i < iterations-1; i++){
        x1 += dx;
        y1 += dy;
        bool occupied = isOccupiedRad(x1, y1, radius);
        if(!occupied){
            // move the point further from obstacle for safety
            foundFree.x = x1 + dx;
            foundFree.y = y1 + dy;
            return true;
        }
    }
    return false;
}


bool Map::getLastNonOccupPos(const Position &freeEdge, const Position &finalSearchPoint, Position &foundLastFree,
                             Position & foundObstacleAt, double radius){
    double dx, dy;
    int iterations;
    double x1 = freeEdge.x;
    double y1 = freeEdge.y;
    getLineIncrements(freeEdge.x, freeEdge.y, finalSearchPoint.x, finalSearchPoint.y, dx, dy, iterations);

    // check points from point freeEdge to finalSearchPoint (excluded)
    for(int i = 0; i < iterations-1; i++){
        x1 += dx;
        y1 += dy;
        bool occupied = isOccupiedRad(x1, y1, radius);
        if(occupied){
            // move the obstacle point further to obstacle
            foundObstacleAt.x = x1+dx;
            foundObstacleAt.y = y1+dx;
            // move the point further from obstacle
            foundLastFree.x = x1-2*dx;
            foundLastFree.y = y1-2*dy;
            return true;
        }
    }
    return false;
}


void Map::getClosestObstaclePoints(const Position pos, double searchDistance, double radius,
                                   std::vector<Position> &obstaclePoints){

    for(auto dir : DIRS){
        Position test(pos);
        // number of iterations needed to check all points
        int iterations = (int)searchDistance/this->resolution;
        for(int i=0; i < iterations; i++){
            test.x += this->resolution*dir[0];
            test.y += this->resolution*dir[1];
            if(isOccupiedRad(test.x,test.y, radius)){
                Position p(test);
                obstaclePoints.push_back(p);
                break;
            }
        }
    }
}


void Map::getMapFreeCornersInM(std::vector<Position> &corners, double radius){
    int rad = (int) radius*100;
    auto freeCorners = freeMapCorners.find(rad);
    if(freeCorners != freeMapCorners.end()){
        corners = freeCorners->second;
        return;
    }

    // find first empty corner
    for(int i = 0; i < map.size(); i++){
        Position p;
        idxToM(i, p);
        if(!isOccupiedRad(p.x, p.y, radius + SAFETY_RANGE)){
            corners.push_back(p);
            break;
        }
    }
    // the other two corners find from back
    for(int i = map.size()-1; i > 0; i--){
        Position p;
        idxToM(i, p);
        if(!isOccupiedRad(p.x, p.y, radius + SAFETY_RANGE)){
            corners.push_back(p);
            break;
        }
    }
    freeMapCorners.emplace(rad, corners);
}


int Map::getIndexToMap(double x, double y) {

    int xIdx = round ((x - origX)/resolution);
    int yIdx = round ((y - origY)/resolution);

    if(xIdx > width || yIdx > height){
        return -1;
    }

    int index = xIdx + width * yIdx;
    return index;
}


int Map::idxToM(int idx, Position &posM){
    if(idx > width * height){
        return -1;
    }
    int yIdx = (int) idx/width;
    int xIdx = idx % width;

    double x = xIdx*resolution + origX;
    double y = yIdx*resolution + origY;
    posM.x = x;
    posM.y = y;

    return 0;
}


void Map::otherRobotFinished(const std::string finishedRobot, const Position robotsFinalPose) {
    finishedRobots[finishedRobot] = robotsFinalPose;
}

double Map::getResolution() {return resolution;}


