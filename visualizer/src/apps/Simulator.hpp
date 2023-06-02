#pragma once

#include "visualizer/rt/Scene.hpp"

struct Simulator {
    Simulator();
    void draw();

private:
    Scene scene;

    bool showTestRay = false;
    Ray testRay;

    int Nsamples = 8*1024;
    bool drawQuadTree = false;
    bool computePdfNorm = false;
    float exposure = -5;
    float gridOpacity = 0.03f;

    bool showImguiDemo = false;

    void saveSVG(const char *filename);
};
