#include "sim/Obstacle.h"

#include <iostream>
#include <string>

namespace {

int failures = 0;

void report(const std::string& name, bool ok) {
    std::cout << name << ": " << (ok ? "PASS" : "FAIL") << "\n";
    if (!ok) ++failures;
}

bool checkA() {
    ObstacleManager m;
    return m.withinBoundary(0, 0, 0) == true
        && m.withinBoundary(10, 0, 0) == false
        && m.withinBoundary(0, 10, 0) == false;
}

bool checkB() {
    ObstacleManager m;
    m.setBoundary(Boundary{-10, 10, -10, 10, 20});
    return m.withinBoundary(5, 5, 5) == true
        && m.withinBoundary(15, 0, 0) == false;
}

bool checkC() {
    ObstacleManager m;
    Obstacle o;
    o.id = "o1";
    o.type = "box";
    o.x = 1; o.y = 0.5; o.z = 0;
    o.sx = 0.5; o.sy = 0.5; o.sz = 0.5;
    m.addObstacle(o);
    return m.checkCollision(1, 0.5, 0, 0.1) == true
        && m.checkCollision(5, 0.5, 0, 0.1) == false;
}

bool checkD() {
    ObstacleManager m;
    Obstacle o;
    o.id = "o1";
    o.type = "box";
    o.x = 1; o.y = 0.5; o.z = 0;
    o.sx = 0.5; o.sy = 0.5; o.sz = 0.5;
    m.addObstacle(o);
    return m.removeObstacle("o1") == true && m.obstacles.empty();
}

bool checkE() {
    ObstacleManager m;
    return m.importObstacleText("anything") == 0 && m.rejectedCount == 0;
}

}

int main() {
    report("(a) default boundary", checkA());
    report("(b) setBoundary", checkB());
    report("(c) addObstacle + checkCollision", checkC());
    report("(d) removeObstacle", checkD());
    report("(e) reject bookkeeping", checkE());

    if (failures == 0) {
        std::cout << "ALL PASS\n";
        return 0;
    }
    std::cout << "FAILED\n";
    return 1;
}
