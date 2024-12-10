#include <PathPlanner.h>

std::vector<Point> PathPlanner::generateSpline(std::vector<Point>& controlPoints, float tension, int segments) {
    std::vector<Point> path;

    // Ensure there are enough points to create splines
    if (controlPoints.size() < 2) {
        return controlPoints; // Not enough points
    }

    // Include the first and last points in the output explicitly
    path.push_back(controlPoints[0]);

    // Add duplicate points at the start and end for padding
    std::vector<Point> extendedPoints = controlPoints;
    extendedPoints.insert(extendedPoints.begin(), controlPoints[0]);
    extendedPoints.push_back(controlPoints.back());

    // Iterate through the control points to calculate spline
    for (size_t i = 1; i < extendedPoints.size() - 2; ++i) {
        Point p0 = extendedPoints[i - 1];
        Point p1 = extendedPoints[i];
        Point p2 = extendedPoints[i + 1];
        Point p3 = extendedPoints[i + 2];

        // Calculate spline points for this segment
        for (int j = 1; j <= segments; ++j) { // Start from 1 to avoid duplicating p1
            float t = j / static_cast<float>(segments); // Parameter t between 0 and 1

            float t2 = t * t;
            float t3 = t2 * t;

            // Catmull-Rom blending functions
            float b0 = -tension * t3 + 2 * tension * t2 - tension * t;
            float b1 = (2 - tension) * t3 + (tension - 3) * t2 + 1;
            float b2 = (tension - 2) * t3 + (3 - 2 * tension) * t2 + tension * t;
            float b3 = tension * t3 - tension * t2;

            // Calculate the interpolated point
            Point interpolated;
            interpolated.y = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
            interpolated.x = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;

            path.push_back(interpolated);
        }
    }

    // Explicitly add the last point to the path
    path.push_back(controlPoints.back());

    return path;
}