#include "SearchGraph.h"


SearchGraph::SearchGraph(std::shared_ptr<Map> map, double robotRadius) {
    this->map = map;
    this->robotRadius = robotRadius;
}

double SearchGraph::getCost(GraphLocation from, GraphLocation to) {
    return utils::compute_euklidean_distance(from.getX(), from.getY(), to.getX(), to.getY());;
}

std::vector<GraphLocation> SearchGraph::neighbors(GraphLocation curNode, GraphLocation goal) const {
    std::vector<GraphLocation> nextNodes;
    for(auto dir : map->DIRS){
        double newX = curNode.getX() + dir[0]*robotRadius;
        double newY = curNode.getY() + dir[1]*robotRadius;

        if(checkFreeLocation(newX, newY, robotRadius)){
            GraphLocation n(newX, newY);
            nextNodes.push_back(n);
        }
    }
    // also try to push goal directly
    if(!map->lineColision(curNode.getX(), curNode.getY(), goal.getX(), goal.getY(), robotRadius)){
        GraphLocation n(goal.getX(), goal.getY());
        nextNodes.push_back(n);
    }
    return nextNodes;
}


bool SearchGraph::checkFreeLocation(double x, double y, double radius) const{

    bool occupied;
    occupied = map->isOccupiedRad(x,y,radius);

    return !occupied;
}


