#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <Arduino.h>
#include <vector>

struct Point {
    float x;
    float y;
};

class PathPlanner {
    public:
        static std::vector<Point> generateSpline(std::vector<Point>& controlPoints, float tension = 0.5, int segments = 10);
};
#endif // PATHPLANNER_H