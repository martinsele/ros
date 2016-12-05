#ifndef TECH_TASK_MAP_H
#define TECH_TASK_MAP_H


#include <vector>
#include <geometry_msgs/Point.h>
#include "unordered_map"
#include <math.h>
#include <utils.h>


struct Position{
    double x, y;

    Position(double xPos=0, double yPos=0){
        x = xPos;
        y = yPos;
    }
};


class Map {
public:

    /** Safety range not to hit obstacles */
    const double SAFETY_RANGE = 0.1;


    Map(const int width, const int height, const double resolution, const std::vector<int8_t> &data, const geometry_msgs::Point &mapOrigin);

    /**
     * Decide about occupancy of a point (x,y) in meters
     * @param x
     * @param y
     * @return
     */
    bool isOccupied(double x, double y);

    /**
     * Check whether a point and its neighborhood is occupied
     * @param x
     * @param y
     * @param radius specify neighbourhood range
     * @return
     */
    bool isOccupiedRad(double x, double y, double radius);

    /**
     * Check whether a line and its neighborhood is occupied
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param radius specify neighbourhood range
     * @return
     */
    bool lineColision(double x1, double y1, double x2, double y2, double radius);


    /**
     * Get obstacle edges at line between fist and last positions
     * @param first
     * @param last
     * @param foundEdges
     * @param radius
     */
    void
    getObstacleEdgesAtLine(const Position &first, const Position &last, std::vector<Position> &foundEdges, double radius);

    /**
     * Find first free position next to an obstacle in line given by positions obstEdge and finalSearchPoint
     * @param obstEdge
     * @param finalSearchPoint
     * @param foundFree
     * @param radius
     * @return true if find such a point
     */
    bool getFirstFreePos(const Position &obstEdge, const Position &finalSearchPoint, Position &foundFree, double radius);

    /**
     * Found last non-occupied position before first obstacle on line between freeEdge and finalSearchPoint
     * @param freeEdge start seach point
     * @param finalSearchPoint last search point
     * @param foundLastFree returned point
     * @param foundObstacleAt position of the obstacle found
     * @param radius robot radius
     * @return true if find such a point
     */
    bool
    getLastNonOccupPos(const Position &freeEdge, const Position &finalSearchPoint, Position &foundLastFree,
                       Position & foundObstacleAt, double radius);

    /**
     * @return map resolution in m/cell
     */
    double getResolution();

    /**
     * Find map corners accessible by robot with given radius
     * @param corners
     * @param radius
     */
    void getMapFreeCornersInM(std::vector<Position> &corners, double radius);

    /**
     * In given distance search for obstacle points
     * @param pos
     * @param searchDistance
     * @param radius
     * @param obstaclePoints
     */
    void getClosestObstaclePoints(const Position pos, double searchDistance, double radius,
                                  std::vector<Position> &obstaclePoints);



    /**
     * Inform robot about finished neighbor robot, avoid its position from now on
     * @param finishedRobot
     * @param robotsFinalPose
     */
    void otherRobotFinished(const std::string finishedRobot, const Position robotsFinalPose);


    const std::vector<std::vector<int>> DIRS = {{1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}};

private:

    std::vector<int8_t> map;
    int width, height;
    double resolution;
    double origX, origY;


    /** corners of free map interior for given radius multiplied by 100 */
    std::unordered_map<int, std::vector<Position>> freeMapCorners;

    std::unordered_map<std::string, Position> finishedRobots;

    /** last position in map */
    Position lastPosition;

    /**
     * Get index to data map
     * @param x
     * @param y
     * @return index to data map, or -1 if x or y out of world
     */
    int getIndexToMap(double x, double y);


    int idxToM(int idx, Position &posM);


    /**
     * Get parameters for map iterations along a line
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param dx - x increment
     * @param dy - y increment
     * @param iterations - number of iterations needed
     */
    void getLineIncrements(double x1, double y1, double x2, double y2, double &dx, double &dy, int &iterations) const;

};



#endif //TECH_TASK_MAP_H
