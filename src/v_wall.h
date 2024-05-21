#ifndef __V_WALL_AGENT__H
#define __V_WALL_AGENT__H 

#include "enviro.h"

using namespace enviro;

class VWallController : public Process, public AgentInterface {

    public:
    VWallController() : Process(), AgentInterface() {}

    void init() {}
    void start() {}
    void update() {}
    void stop() {}

};

class VWall : public Agent {
    public:
    VWall(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    VWallController c;
};

DECLARE_INTERFACE(VWall)

#endif