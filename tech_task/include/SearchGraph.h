#ifndef TECH_TASK_SEARCHGRAPH_H
#define TECH_TASK_SEARCHGRAPH_H

#include "Map.h"
#include "GraphLocation.h"


class SearchGraph{

public:
    SearchGraph(std::shared_ptr<Map> map, double robotRadius);

    double getCost(GraphLocation from, GraphLocation to);

    /**
     * Generate neighbors from location
     * @param loc
     * @param goal
     * @return
     */
    std::vector<GraphLocation> neighbors(GraphLocation loc, GraphLocation goal) const;


private:

    std::shared_ptr<Map> map;
    double robotRadius;


    /**
     * Check position for map and neighbors
     * @param x
     * @param y
     * @param radius
     * @return
     */
    bool checkFreeLocation(double x, double y, double radius) const;

};

#endif //TECH_TASK_SEARCHGRAPH_H
