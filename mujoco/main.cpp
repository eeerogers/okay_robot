
#include <GLFW/glfw3.h>
#include <iostream>
#include <memory>

#include "build_with_robots/mujoco_gui.hpp"
#include "mujoco/mujoco.h"

const char* XML_PATH = "/home/eric/projects/mujoco_menagerie/stanford_tidybot/scene.xml";
const int errstr_size = 500;
const bool enable_gui = true;

int main()
{
    mjModel* m = nullptr;
    mjData* d = nullptr;

    char errstr[errstr_size];
    m = mj_loadXML(XML_PATH, NULL, errstr, errstr_size);
    if (!m) {
        printf("model not loaded: %s\n", errstr);
        return 0;
    }
    d = mj_makeData(m);

    MujocoGUI mujoco_gui(&m, &d);
    if (!mujoco_gui.init()) {
        printf("error initializing mujoco gui\n");
        return 0;
    }

    while (!mujoco_gui.should_close()) {
        mj_step(m, d);
        mujoco_gui.update();
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    mujoco_gui.free();

    return 0;
}