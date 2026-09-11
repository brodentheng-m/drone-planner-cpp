#ifndef DRONEPLANNERCPP_SIM_OBSTACLE_H
#define DRONEPLANNERCPP_SIM_OBSTACLE_H

#include <string>
#include <vector>

struct Obstacle {
    std::string id;
    std::string type;
    double x = 0, y = 0, z = 0;
    double sx = 0, sy = 0, sz = 0;
    double radius = 0;
    double height = 0;
    int r = 255, g = 0, b = 0;
    std::string label;
};

struct Boundary {
    double minX = -4, maxX = 4, minZ = -4, maxZ = 4, maxY = 4;
};

class ObstacleManager {
public:
    std::vector<Obstacle> obstacles;
    Boundary boundary;
    int rejectedCount = 0;

    void setBoundary(const Boundary& b);
    Boundary getBoundary() const;
    void addObstacle(const Obstacle& o);
    bool removeObstacle(const std::string& id);
    void clear();
    bool checkCollision(double x, double y, double z, double halfSize) const;
    bool withinBoundary(double x, double y, double z) const;
    int importObstacleText(const std::string& text);
};

#endif
