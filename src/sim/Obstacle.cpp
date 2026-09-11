#include "sim/Obstacle.h"

void ObstacleManager::setBoundary(const Boundary& b) {
    boundary = b;
}

Boundary ObstacleManager::getBoundary() const {
    return boundary;
}

void ObstacleManager::addObstacle(const Obstacle& o) {
    obstacles.push_back(o);
}

bool ObstacleManager::removeObstacle(const std::string& id) {
    for (auto it = obstacles.begin(); it != obstacles.end(); ++it) {
        if (it->id == id) {
            obstacles.erase(it);
            return true;
        }
    }
    return false;
}

void ObstacleManager::clear() {
    obstacles.clear();
    rejectedCount = 0;
}

bool ObstacleManager::withinBoundary(double x, double y, double z) const {
    return x >= boundary.minX && x <= boundary.maxX
        && z >= boundary.minZ && z <= boundary.maxZ
        && y >= 0 && y <= boundary.maxY;
}

bool ObstacleManager::checkCollision(double x, double y, double z, double halfSize) const {
    const double pMinX = x - halfSize, pMaxX = x + halfSize;
    const double pMinY = y - halfSize, pMaxY = y + halfSize;
    const double pMinZ = z - halfSize, pMaxZ = z + halfSize;

    for (const Obstacle& o : obstacles) {
        double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
        if (o.type == "sphere") {
            oMinX = o.x - o.radius; oMaxX = o.x + o.radius;
            oMinY = o.y - o.radius; oMaxY = o.y + o.radius;
            oMinZ = o.z - o.radius; oMaxZ = o.z + o.radius;
        } else if (o.type == "cylinder") {
            const double hh = o.height / 2.0;
            oMinX = o.x - o.radius; oMaxX = o.x + o.radius;
            oMinY = o.y - hh;       oMaxY = o.y + hh;
            oMinZ = o.z - o.radius; oMaxZ = o.z + o.radius;
        } else {
            oMinX = o.x - o.sx; oMaxX = o.x + o.sx;
            oMinY = o.y - o.sy; oMaxY = o.y + o.sy;
            oMinZ = o.z - o.sz; oMaxZ = o.z + o.sz;
        }
        const bool overlap =
            pMinX <= oMaxX && pMaxX >= oMinX &&
            pMinY <= oMaxY && pMaxY >= oMinY &&
            pMinZ <= oMaxZ && pMaxZ >= oMinZ;
        if (overlap) return true;
    }
    return false;
}

int ObstacleManager::importObstacleText(const std::string& text) {
    (void)text;
    return 0;
}
