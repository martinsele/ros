#ifndef TECH_TASK_LOCATION_H
#define TECH_TASK_LOCATION_H

#include "utils.h"

class GraphLocation;

namespace std {
    template<> struct hash<GraphLocation> {
        size_t operator()(const GraphLocation &node) const;
    };
}

class GraphLocation{
public:
    GraphLocation(double x, double y);
    GraphLocation();

    double getX() const;
    double getY() const;

    size_t operator()(const GraphLocation & node) const;

    bool operator==(const GraphLocation & other) const;

    bool operator< (const GraphLocation & other) const;

    bool isSame(GraphLocation location);

private:
    double x, y;
};

/** Minimal distance to consider locations equal */
const double MIN_EQ_LOC_DIST = 0.1;



#endif //TECH_TASK_LOCATION_H
