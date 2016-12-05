#include "GraphLocation.h"

GraphLocation::GraphLocation(double x, double y) {
    this->x = x;
    this->y = y;
}


GraphLocation::GraphLocation() {
    this->x = 0;
    this->y = 0;
}


size_t GraphLocation::operator()(const GraphLocation & node) const{
    size_t res = 17;
    res = res * 31 + (int)(node.x*MIN_EQ_LOC_DIST);
    res = res * 1111 + (int)(node.y*MIN_EQ_LOC_DIST);
    return res;
}

bool GraphLocation::operator==(const GraphLocation & other) const {
    // if two nodes are close enough, consider them equal
    return (std::abs(x-other.x) < MIN_EQ_LOC_DIST && std::abs(y-other.y) < MIN_EQ_LOC_DIST);
}

double GraphLocation::getX() const{
    return x;
}

double GraphLocation::getY() const{
    return y;
}

bool GraphLocation::isSame(GraphLocation location) {
    // if two nodes are close enough, consider them equal
    return (std::abs(x-location.x) < MIN_EQ_LOC_DIST && std::abs(y-location.y) < MIN_EQ_LOC_DIST);
}

bool GraphLocation::operator<(const GraphLocation &other) const {
    if(x < other.x){
        return true;
    }else if(x == other.x){
        return (y < other.y);
    }
    return false;
}



size_t std::hash<GraphLocation>::operator()(const GraphLocation &node) const {
    size_t res = 17;
    res = res * 31 + (int)(node.getX()*MIN_EQ_LOC_DIST);
    res = res * 1111 + (int)(node.getY()*MIN_EQ_LOC_DIST);
    return res;
}
