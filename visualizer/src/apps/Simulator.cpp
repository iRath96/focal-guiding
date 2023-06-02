#include "Simulator.hpp"
#include "imgui.h"
#include "../viridis.hpp"
#include "../tinyexr.h"

void app() {
    static Simulator app;
    app.draw();
}

static ImColor sensorColors[] = {
    ImColor(255, 0, 0),
    ImColor(255, 255, 0),
    ImColor(0, 255, 255),
    ImColor(0, 255, 0),
    ImColor(0, 0, 255)
};

Scene makeDoubleSlit(Float gap) {
    Scene scene;

    scene.addLine({{0, 0}, {0, 1}, 0});
    scene.addLine({{0, 1}, {1, 1}, 0});
    scene.addLine({{1, 1}, {1, 0}, 0});
    scene.addLine({{1, 0}, {0, 0}, 1});

    scene.addLine({{0, 0.3}, {Float(0.4) - gap, 0.3}, 0});
    scene.addLine({{Float(0.4) + gap, 0.3}, {Float(0.6) - gap, 0.3}, 0});
    scene.addLine({{Float(0.6) + gap, 0.3}, {1, 0.3}, 0});

    scene.addSensor({{0.101, 0.801}});
    scene.addSensor({{0.301, 0.801}});
    scene.addSensor({{0.501, 0.801}});
    scene.addSensor({{0.701, 0.801}});
    scene.addSensor({{0.901, 0.801}});

    return scene;
}

Scene makePinholeScene(Float gap) {
    Scene scene;

    scene.addLine({{0, 0}, {0, 1}, 0});
    scene.addLine({{0, 1}, {1, 1}, 0});
    scene.addLine({{1, 1}, {1, 0}, 0});
    scene.addLine({{1, 0}, {0, 0}, 1});

    scene.addLine({{0, 0.3}, {Float(0.5) - gap, 0.3}, 0});
    scene.addLine({{Float(0.5) + gap, 0.3}, {1, 0.3}, 0});

    scene.addSensor({{0.101, 0.801}});
    scene.addSensor({{0.301, 0.801}});
    scene.addSensor({{0.501, 0.801}});
    scene.addSensor({{0.701, 0.801}});
    scene.addSensor({{0.901, 0.801}});

    return scene;
}

Scene makeReflectedPinholeScene(Float gap) {
    Scene scene;

    scene.addLine({{0, 0}, {1, 0}, 1});

    Line floor{{0.2, 0.5}, {0.8, 0.5}, 0};
    floor.reflect = true;
    scene.addLine(floor);

    scene.addLine({{0, 0.25}, {Float(0.5) - gap, 0.25}, 0});
    scene.addLine({{Float(0.5) + gap, 0.25}, {1, 0.25}, 0});

    scene.addSensor({{0.1, 0.3}});

    scene.aabb().max = Vec2f { 1, 0.75 };

    return scene;
}

Scene makeCascadeScene() {
    const Float gap = 0.1;

    Scene scene;

    scene.addLine({{0, 0}, {0, 1}, 0});
    scene.addLine({{0, 1}, {1, 1}, 0});
    scene.addLine({{1, 1}, {1, 0}, 0});
    scene.addLine({{1, 0}, {0, 0}, 1});

    for (int i = 0; i <= 1; i++) {
        const Float y = Float(0.2) + Float(0.2) * Float(i);
        scene.addLine({{0, y}, {Float(0.5) - gap, y}, 0});
        scene.addLine({{Float(0.5) + gap, y}, {1, y}, 0});
    }

    scene.addSensor({{0.101, 0.801}});
    scene.addSensor({{0.301, 0.801}});
    scene.addSensor({{0.501, 0.801}});
    scene.addSensor({{0.701, 0.801}});
    scene.addSensor({{0.901, 0.801}});

    return scene;
}

Scene makeCrossedScene() {
    Scene scene;

    const Float p0 = 0.00;
    const Float p1 = 0.45;
    const Float p2 = 0.55;
    const Float p3 = 1.00;

    scene.addLine({{p0, p1}, {p1, p1}, 0});
    scene.addLine({{p1, p1}, {p1, p0}, 0});
    scene.addLine({{p1, p0}, {p2, p0}, 1});
    scene.addLine({{p2, p0}, {p2, p1}, 0});
    scene.addLine({{p2, p1}, {p3, p1}, 0});
    scene.addLine({{p3, p1}, {p3, p2}, 0});
    scene.addLine({{p3, p2}, {p2, p2}, 0});
    scene.addLine({{p2, p2}, {p2, p3}, 0});
    scene.addLine({{p2, p3}, {p1, p3}, 0});
    scene.addLine({{p1, p3}, {p1, p2}, 0});
    scene.addLine({{p1, p2}, {p0, p2}, 0});
    scene.addLine({{p0, p2}, {p0, p1}, 1});

    scene.addSensor({{0.95, 0.5}});
    scene.addSensor({{0.5, 0.95}});

    /*scene.addLine({{1.2,0},{1.3,0}, 0.5});
    scene.addLine({{1.3,0},{1.3,1}, 0});
    scene.addLine({{1.3,1},{1.2,1}, 0});
    scene.addLine({{1.2,1},{1.2,0}, 0});

    scene.addSensor({{1.25, 0.95}});

    scene.addLine({{1.4,0},{1.5,0}, 0.1});
    scene.addLine({{1.5,0},{1.5,1}, 0});
    scene.addLine({{1.5,1},{1.4,1}, 0});
    scene.addLine({{1.4,1},{1.4,0}, 0});

    scene.addSensor({{1.45, 0.45}});*/

    return scene;
}

Simulator::Simulator() {
    scene = makeReflectedPinholeScene(0.01);
    testRay.d = {0, 1};
}

void Simulator::saveSVG(const char *filename) {
    std::ofstream file{filename};

    std::vector<focal_guiding::Distribution<Env2D>::Patch> patches;
    patches = scene.quadTree.visualize();

    const float scale = 500;
    file << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"512\" height=\"512\" "
        << "style=\"stroke-opacity: 0.2; background: rgb(48, 18, 59);\" "
        << "viewBox=\""
        << scale * scene.aabb().min.x() << " "
        << scale * scene.aabb().min.y() << " "
        << scale * scene.aabb().diagonal().x() << " "
        << scale * scene.aabb().diagonal().y()
        << "\">" << std::endl;

    for (int step = 0; step < 2; step++) {
        file << "  <g>" << std::endl;

        const float alphaNorm = pow(float(2), exposure);
        for (const auto &element: patches) {
            const auto v = turbo((log2(alphaNorm * element.density + 1.0f / 32) + 5) / 10.f);
            const ImColor color{v.r, v.g, v.b, 1.0f};

            const auto pos = element.domain.min;
            const auto size = element.domain.diagonal();
            file << "    <rect "
                 << "x=\"" << scale * pos.x() << "\" y=\"" << scale * pos.y() << "\" "
                 << "width=\"" << scale * size.x() << "\" height=\"" << scale * size.y() << "\" ";

            if (step == 0) {
                file
                    << "fill=\"rgb(" << int(255 * v.r) << ", " << int(255 * v.g) << ", " << int(255 * v.b) << ")\" "
                    << "stroke=\"none\" ";
            } else {
                file
                    << "fill=\"none\" "
                    << "stroke=\"#ffffff\" ";
            }

            file << "/>" << std::endl;
        }

        file << "  </g>" << std::endl;
    }

    file << "</svg>" << std::endl;
}

void Simulator::draw() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Experiments")) {
            if (ImGui::MenuItem("Figure 3")) scene = makeReflectedPinholeScene(0.01);
            if (ImGui::MenuItem("Crossroad")) scene = makeCrossedScene();
            if (ImGui::MenuItem("Camera obscura (simple)")) scene = makePinholeScene(0.1);
            if (ImGui::MenuItem("Camera obscura (medium)")) scene = makePinholeScene(0.01);
            if (ImGui::MenuItem("Camera obscura (hard)")) scene = makePinholeScene(0.001);
            if (ImGui::MenuItem("Double-slit")) scene = makeDoubleSlit(0.01);
            if (ImGui::MenuItem("Cascade")) scene = makeCascadeScene();
            ImGui::EndMenu();
        }

        //if (ImGui::BeginMenu("Demos")) {
        //    ImGui::MenuItem("ImGui demo", nullptr, &showImguiDemo);
        //    ImGui::EndMenu();
        //}

        ImGui::EndMainMenuBar();
    }

    if (showImguiDemo) ImGui::ShowDemoWindow(&showImguiDemo);

    if (ImGui::Begin("Viewport")) {
        ImVec2 scale = ImGui::GetContentRegionAvail();
        scale.x /= scene.aabb().diagonal().x();
        scale.y /= scene.aabb().diagonal().y();
        scale.x = scale.y = std::min(scale.x, scale.y);

        ImVec2 shift = ImGui::GetCursorScreenPos();
        shift.x -= scene.aabb().min.x() * scale.x;
        shift.y -= scene.aabb().min.y() * scale.y;

        const auto transform = [&](const Vec2f &p) {
            return ImVec2(p.x() * scale.x + shift.x, p.y() * scale.y + shift.y);
        };

        auto &drawList = *ImGui::GetWindowDrawList();

        const ImVec2 cursorPos = ImGui::GetMousePos();
        const ImVec2 invCursorPos = {(cursorPos.x - shift.x) / scale.x, (cursorPos.y - shift.y) / scale.y};

        const float alphaNorm = pow(float(2), exposure);
        double norm = 0;
        const auto patches = scene.quadTree.visualize();

        Float pdf = 0;
        for (const auto &element: patches) {
            norm += element.domain.area() * element.density;
            const auto v = turbo((log2(alphaNorm * element.density + 1.0f / 32) + 5) / 10.f);
            const ImColor color { v.r, v.g, v.b, 1.0f };
            //const ImColor color = ImColor(1.f, 1.f, 1.f, alphaNorm * element.density);
            drawList.AddRectFilled(transform(element.domain.min), transform(element.domain.max), color);
            drawList.AddRect(transform(element.domain.min), transform(element.domain.max), ImColor(1.f, 1.f, 1.f, gridOpacity));

            if (
                invCursorPos.x >= element.domain.min[0] &&
                invCursorPos.x <= element.domain.max[0] &&
                invCursorPos.y >= element.domain.min[1] &&
                invCursorPos.y <= element.domain.max[1]) {
                pdf = element.density;
            }
        }

        for (auto &line: scene.lines()) {
            ImColor color = IM_COL32_WHITE;
            if (!line.reflect) color = ImColor(1.f, 0.f, 0.f);
            if (line.emission > 0) color = ImColor(1.f, 0.5f, 0.f);
            drawList.AddLine(transform(line.start), transform(line.start + line.span), color, 2);
        }

        for (size_t i = 0; i < scene.sensors().size(); i++) {
            auto &sensor = scene.sensors()[i];
            auto color = sensorColors[i % (sizeof(sensorColors) / sizeof(*sensorColors))];
            drawList.AddCircleFilled(transform(sensor.position), 4, color);
        }

        if (computePdfNorm) {
            ImGui::Text("spatial pdf norm: %lf", norm);
        }

        if (ImGui::IsWindowHovered() && scene.guidingReady) {
            ImGui::Text("Position: %.3f %.3f", invCursorPos.x, invCursorPos.y);
            ImGui::Text("Spatial PDF: %.3e", pdf);
        }

        if (showTestRay) {
            drawList.AddCircleFilled(transform(testRay.o), 4, ImColor(1.f, 1.f, 1.f));
            drawList.AddLine(transform(testRay(0)), transform(testRay(1)), ImColor(1.f, 1.f, 1.f, 0.5f));
        }
    }
    ImGui::End();

    if (ImGui::Begin("Sensors")) {
        ImGui::SeparatorText("Help");
        ImGui::TextWrapped(
            "2-D Focal Path Guiding Simulator:\n"
            "Switch between scenes from the 'Experiments' menu in the top left. "
            "Orange lines denote light sources, white lines are diffusely reflecting walls, and red lines block light. "
            "The small colorful circles are sensors that capture light from all directions."
        );

        ImGui::SeparatorText("Visualization Settings");
        ImGui::DragFloat("Exposure", &exposure, 0.01f, -20, +20);
        ImGui::DragFloat("Grid Opacity", &gridOpacity, 0.001f, 0, 1);
        
        ImGui::SeparatorText("Statistics");

        Float totalV = 0;
        for (size_t i = 0; i < scene.sensors().size(); i++) {
            auto &sensor = scene.sensors()[i];
            auto color = sensorColors[i % (sizeof(sensorColors) / sizeof(*sensorColors))];
            ImGui::TextColored(color, "Sensor #%lu", i);
            ImGui::Text("estimate = %.3e", sensor.estimate);
            ImGui::Text("variance = %.3e", sensor.variance);
            totalV += sensor.variance;
        }

        ImGui::Separator();
        ImGui::Text("variance sum = %.3e", totalV);

        ImGui::SeparatorText("Simulation");
        ImGui::Text("Simulate one iteration using...");

        if (ImGui::Button("Uniform sampling")) {
            scene.simulate(Nsamples);
        }

        ImGui::BeginDisabled(!scene.guidingReady);
        ImGui::SameLine();
        if (ImGui::Button("Guided sampling")) {
            scene.simulateGuided(Nsamples);
        }
        ImGui::EndDisabled();

#ifndef __EMSCRIPTEN__
        if (ImGui::Button("Save SVG")) {
            saveSVG("quadtree.svg");
        }
#endif

        ImGui::InputInt("#Samples", &Nsamples);

        ImGui::SeparatorText("Guiding Settings");
        {//if (ImGui::CollapsingHeader("Guiding Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextWrapped("Use the following settings during the next iteration:");
            ImGui::Checkbox("Iterative Narrowing", &scene.iterativeNarrowing);
            ImGui::Checkbox("Prune Guiding Structure", &scene.quadTree.configuration.pruning);
            ImGui::Checkbox("Splat Camera Rays", &scene.splatCamera);
            ImGui::DragFloat("Split Thresh.", &scene.quadTree.configuration.threshold, scene.quadTree.configuration.threshold / 100, 1e-5, 1e-1, "%.1e");
        }

        ImGui::SeparatorText("Probe Ray");
        {//if (ImGui::CollapsingHeader("Probe Ray", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Show Probe", &showTestRay);
            if (showTestRay) {
                ImGui::DragFloat("Origin X", &testRay.o.x(), 0.001f);
                ImGui::DragFloat("Origin Y", &testRay.o.y(), 0.001f);

                float angle = std::atan2(testRay.d.y(), testRay.d.x());
                if (ImGui::DragFloat("Angle", &angle, 0.001f)) {
                    testRay.d.x() = std::cos(angle);
                    testRay.d.y() = std::sin(angle);
                }

                if (scene.guidingReady) {
                    ImGui::Text("Directional PDF: %.3e", scene.quadTree.pdf(testRay.o, testRay.d));
                    /*ImGui::Checkbox("Check normalization", &computePdfNorm);
                    if (computePdfNorm) {
                        const int N = 8192;
                        double acc = 0;
                        for (int i = 0; i < N; i++) {
                            const Float angle = Float(2.0 * M_PI / N) * i;
                            Ray r;
                            r.o = testRay.o;
                            r.d = {std::cos(angle), std::sin(angle)};
                            acc += scene.quadTree.pdf(r.o, r.d);
                        }
                        ImGui::Text("norm %f", Float(acc / N));
                    }*/
                } else {
                    ImGui::Text("(Guiding has not yet been trained)");
                }
            }
        }
    }
    ImGui::End();
}
