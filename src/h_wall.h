#ifndef __H_WALL_AGENT__H
#define __H_WALL_AGENT__H 

#include "enviro.h"

using namespace enviro;

class HWallController : public Process, public AgentInterface {

    public:
    HWallController() : Process(), AgentInterface() {}

    void init() {}
    void start() {}
    void update() {}
    void stop() {}

};

class HWall : public Agent {
    public:
    HWall(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    HWallController c;
};

DECLARE_INTERFACE(HWall)

#endif