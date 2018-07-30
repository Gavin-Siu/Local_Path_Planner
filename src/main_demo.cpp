
#define DEBUG

#include <ros/ros.h>
#include <ros/package.h>
#include <pathfinder/pathfinder.h>

using namespace ros;


int main(int argc, char **argv) {
    init(argc, argv, "pathfinder-demo");
    NodeHandle n;
    pathfinder *pf_ptr = new pathfinder(n);
    pf_ptr->Config();
    pf_ptr->PrintConfig();
    pf_ptr->Start();

}