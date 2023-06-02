#include "Visualizer.hpp"
#include "../viridis.hpp"

#include <imgui.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>

static const char* AvailableScenes[] = {
    "camera-obscura",
    "dining-room",
    "living-room",
    "modern-hall",
    "modern-living-room"
};

static const char* SceneNames[] = {
    "Camera Obscura",
    "Dining Room",
    "Funky Living Room",
    "Modern Hall",
    "Modern Living Room"
};

static const int SceneAxis[] = {
    1, 2, 1, 2, 2
};

static const float SceneThreshold[] = {
    1.399f, -1.933f, 2.271f, -2.941f, -0.113f
};

void app() {
    static Visualizer app;
    app.draw();
}

Mesh::Mesh(std::istream &file) {
    loadPly(file);
}

Mesh::Slice Mesh::slice(int dim, float value) const {
    // preparations
    using VertexIndex = int;
    using SegmentIndex = int;
    using Edge = std::pair<VertexIndex, VertexIndex>;

    struct Segment {
        std::array<Edge, 2> edges;
        bool emitted = false;
    };

    std::map<Edge, std::vector<SegmentIndex>> edgeSegments;
    std::vector<Segment> segments;

    for (const auto &triangle: triangles) {
        const int numEdges = 3;

        Segment segment;
        int sIndex = 0;

        VertexIndex prevVertex = triangle[numEdges - 1];
        bool prevCheck = vertices[prevVertex][dim] < value;
        for (int edge = 0; edge < numEdges; edge++) {
            VertexIndex curVertex = triangle[edge];
            bool curCheck = vertices[curVertex][dim] < value;

            if (prevCheck != curCheck) {
                // edge of face crosses slicing plane
                segment.edges[sIndex++] = prevVertex < curVertex ?
                                          std::make_pair(prevVertex, curVertex) :
                                          std::make_pair(curVertex, prevVertex);

                if (sIndex >= 2) {
                    // completed a segment
                    const auto segmentIndex = (SegmentIndex) segments.size();
                    segments.push_back(segment);

                    for (const auto &segmentEdge: segment.edges) {
                        edgeSegments[segmentEdge].push_back(segmentIndex);
                    }

                    sIndex = 0;
                }
            }

            prevVertex = curVertex;
            prevCheck = curCheck;
        }
    }

    const auto intersectEdge = [&](const Edge &edge) {
        const auto v0 = vertices[edge.first];
        const auto v1 = vertices[edge.second];
        const float t = (value - v0[dim]) / (v1[dim] - v0[dim]);

        Slice::Point result;
        for (int i = 0; i < 2; i++) {
            const int d = i < dim ? i : i + 1;
            result[i] = (1 - t) * v0[d] + t * v1[d];
        }
        return result;
    };

    Slice result;
    for (auto &segment: segments) {
        if (segment.emitted) continue;

        segment.emitted = true;

        const size_t prevPointCount = result.points.size();
        result.points.emplace_back(intersectEdge(segment.edges[0]));

        Edge nextEdge = segment.edges[1];
        while (true) {
            const auto &connections = edgeSegments[nextEdge];

            SegmentIndex nextSegmentIndex;
            for (auto &segmentCandidateIndex: connections) {
                if (!segments[segmentCandidateIndex].emitted) {
                    nextSegmentIndex = segmentCandidateIndex;
                    goto foundFollowupSegment;
                }
            }

            // no follow-up segment, the loop is done
            break;

            foundFollowupSegment:
            // emit next segment
            auto &nextSegment = segments[nextSegmentIndex];
            nextSegment.emitted = true;

            result.points.emplace_back(intersectEdge(nextEdge));
            nextEdge = nextSegment.edges[0] == nextEdge ? nextSegment.edges[1] : nextSegment.edges[0];
        }

        result.points.emplace_back(intersectEdge(nextEdge));
        result.polygons.emplace_back(result.points.size() - prevPointCount);
    }

    return result;
}

void Mesh::loadPly(std::istream& stream) {
    std::string magic;
    stream >> magic;
    if (magic != "ply") {
        std::cerr << "Given file is not a ply file." << std::endl;
        return;
    }

    // Read header
    size_t vertexCount = 0;
    size_t faceCount = 0;

    for (std::string line; std::getline(stream, line);) {
        std::stringstream sstream(line);

        std::string action;
        sstream >> action;
        if (action == "comment")
            continue;
        else if (action == "format") {
            continue;
        } else if (action == "element") {
            std::string type;
            sstream >> type;
            if (type == "vertex")
                sstream >> vertexCount;
            else if (type == "face")
                sstream >> faceCount;
        } else if (action == "property") {
            continue; // TODO
        } else if (action == "end_header")
            break;
    }

    // Read entries
     const auto readFloat = [&]() {
        float val = 0;
        stream.read(reinterpret_cast<char*>(&val), sizeof(val));
        return val;
    };

    const auto readIdx = [&]() {
        uint32_t val = 0;
        stream.read(reinterpret_cast<char*>(&val), sizeof(val));
        return val;
    };

    vertices.reserve(vertexCount);
    for (int i = 0; i < vertexCount; ++i) {
        float x = readFloat();
        float y = readFloat();
        float z = readFloat();

        vertices.emplace_back(Vertex{x, y, z});
    }

    if (vertices.empty()) {
        std::cerr << "No vertices found in ply file" << std::endl;
        return; // Failed
    }

    triangles.reserve(faceCount);
    for (int i = 0; i < faceCount; ++i) {
        uint8_t elems = 0;
        stream.read(reinterpret_cast<char*>(&elems), sizeof(elems));

        assert(elems == 3);

        int i0 = (int)readIdx();
        int i1 = (int)readIdx();
        int i2 = (int)readIdx();

        triangles.emplace_back(Triangle{i0, i1, i2});
    }
}

Visualizer::Visualizer() {
    scene = 0;
    req_scene = 0;
    loadScene(scene);
}

void Visualizer::unloadScene() {
    std::vector<Mesh::Vertex>().swap(mesh.vertices);
    std::vector<Mesh::Triangle>().swap(mesh.triangles);
    octree = focal_guiding::Orthtree<Env3D>();
    
    std::vector<Patch>().swap(patches);
    slice = Mesh::Slice();
}

void Visualizer::loadScene(size_t id) {
    const std::string scene_id = AvailableScenes[id];
    sliceAxis = SceneAxis[id];
    sliceValue = SceneThreshold[id];

    std::cout << "reading mesh..." << std::endl;
    std::ifstream meshFile("./data/" + scene_id + ".ply", std::ios::in | std::ios::binary);
    assert(meshFile.is_open());

    mesh = Mesh{meshFile};
    std::cout << "  num vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "  num triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "...done!" << std::endl;

    std::ifstream file{"./data/" + scene_id + "-conv.nose", std::ios::binary};
    assert(file.is_open());
    octree.load(file);
    octree.describe(std::cout);

    volumeNorm = octree.aabb().volume();

    updateGrid();
}

void Visualizer::saveSVG(const char *filename) {
    std::ofstream file{filename};

    const auto &aabb = octree.aabb();

    const int displayAxis[2] = {
        0 >= sliceAxis ? 1 : 0,
        1 >= sliceAxis ? 2 : 1
    };

    const float scale = 100;
    file << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
         << "width=\"512\" height=\"512\" "
         << "style=\"background: rgb(48, 18, 59);\" "
         << "viewBox=\""
         << scale * aabb.min[displayAxis[0]] << " "
         << scale * aabb.min[displayAxis[1]] << " "
         << scale * aabb.diagonal()[displayAxis[0]] << " "
         << scale * aabb.diagonal()[displayAxis[1]]
         << "\">" << std::endl;

    const float drOffset = std::exp2(-dynamicRange/2);
    for (int step = 0; step < 2; step++) {
        file << "  <g style=\"stroke-opacity: 0.2;\">" << std::endl;

        const float alphaNorm = volumeNorm * pow(float(2), exposure);
        for (const auto &element: patches) {
            const auto v = turbo((log2(alphaNorm * element.density + drOffset) + dynamicRange/2) / dynamicRange);
            const ImColor color{v.r, v.g, v.b, 1.0f};

            const auto pos = element.domain.min;
            const auto size = element.domain.diagonal();
            file << "    <rect "
                 << "x=\"" << scale * pos[displayAxis[0]] << "\" y=\"" << scale * pos[displayAxis[1]] << "\" "
                 << "width=\"" << scale * size[displayAxis[0]] << "\" height=\"" << scale * size[displayAxis[1]] << "\" ";

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

    file << "  <g style=\"stroke-width: 4;\">" << std::endl;
    int pointIndex = 0;
    for (auto pointCount: slice.polygons) {
        file << "    <path stroke=\"white\" fill=\"transparent\" d=\"";
        for (int i = 0; i < pointCount; i++) {
            const auto &point = slice.points[pointIndex++];
            file
                << (i ? " L " : "M ")
                << scale * point[0] << " "
                << scale * point[1];
        }
        file << "\" />" << std::endl;
    }
    file << "  </g>" << std::endl;

    file << "</svg>" << std::endl;
}

void Visualizer::drawGrid() {
    const int displayAxis[2] = {
        0 >= sliceAxis ? 1 : 0,
        1 >= sliceAxis ? 2 : 1
    };

    const ImVec2 viewSize = ImGui::GetContentRegionAvail();
    const ImVec2 viewCenter = {viewSize.x / 2, viewSize.y / 2};
    const ImVec2 aabbCenter = {octree.aabb().midpoint()[displayAxis[0]], octree.aabb().midpoint()[displayAxis[1]]};

    ImVec2 scale = ImGui::GetContentRegionAvail();
    scale.x /= octree.aabb().diagonal()[displayAxis[0]];
    scale.y /= octree.aabb().diagonal()[displayAxis[1]];
    scale.x = scale.y = std::min(scale.x, scale.y) * viewScale;

    if (sliceAxis == 2) {
        scale.y *= -1;
    }

    ImVec2 shift = ImGui::GetCursorScreenPos();
    shift.x += viewCenter.x - aabbCenter.x * scale.x + viewShift[0];
    shift.y += viewCenter.y - aabbCenter.y * scale.y + viewShift[1];

    const ImVec2 cursorPos = ImGui::GetMousePos();
    const ImVec2 invCursorPos = {(cursorPos.x - shift.x) / scale.x, (cursorPos.y - shift.y) / scale.y};

    float pdf = 0;

    const auto transform = [&](const Env3D::Vector &p) {
        return ImVec2(p[displayAxis[0]] * scale.x + shift.x, p[displayAxis[1]] * scale.y + shift.y);
    };

    const float alphaNorm = volumeNorm * pow(float(2), exposure);
    auto &drawList = *ImGui::GetWindowDrawList();
    const float drOffset = std::exp2(-dynamicRange/2);
    for (const auto &element: patches) {
        //const ImColor color = ImColor(1.f, 1.f, 1.f, alphaNorm * element.density);
        const auto v = turbo((log2(alphaNorm * element.density + drOffset) + dynamicRange/2) / dynamicRange);
        const ImColor color{v.r, v.g, v.b, 1.0f};
        drawList.AddRectFilled(transform(element.domain.min), transform(element.domain.max), color);

        if (gridOutlineOpacity > 0) {
            drawList.AddRect(transform(element.domain.min), transform(element.domain.max),
                             ImColor(1.f, 1.f, 1.f, gridOutlineOpacity));
        }


        if (
            invCursorPos.x >= element.domain.min[displayAxis[0]] &&
            invCursorPos.x <= element.domain.max[displayAxis[0]] &&
            invCursorPos.y >= element.domain.min[displayAxis[1]] &&
            invCursorPos.y <= element.domain.max[displayAxis[1]]) {
            pdf = element.density;
        }
    }

    // MARK: draw slice
    static std::vector<ImVec2> slicePoints;
    slicePoints.clear();
    slicePoints.reserve(slice.points.size());

    for (auto &point: slice.points) {
        slicePoints.emplace_back(point[0] * scale.x + shift.x, point[1] * scale.y + shift.y);
    }

    int offset = 0;
    for (const auto &polygon: slice.polygons) {
        drawList.AddPolyline(slicePoints.data() + offset, polygon, IM_COL32_WHITE, ImDrawFlags_None, 1);
        offset += polygon;
    }

    if (ImGui::IsWindowHovered()) {
        float pos[3] = {};
        pos[displayAxis[0]] = invCursorPos.x;
        pos[displayAxis[1]] = invCursorPos.y;
        pos[sliceAxis] = sliceValue;

        ImGui::Text("Position: %f %f %f", pos[0], pos[1], pos[2]);
        ImGui::Text("Spatial PDF: %.3e", pdf);

        const auto mouseWheel = ImGui::GetIO().MouseWheel;
        if (mouseWheel != 0) {
            float factor = std::exp(mouseWheel * 0.1f);
            viewScale *= factor;

            viewShift[0] += (shift.x + aabbCenter.x * scale.x - cursorPos.x) * (factor - 1);
            viewShift[1] += (shift.y + aabbCenter.y * scale.y - cursorPos.y) * (factor - 1);
        }

        viewShift[0] += ImGui::GetMouseDragDelta().x;
        viewShift[1] += ImGui::GetMouseDragDelta().y;
        ImGui::ResetMouseDragDelta();
    }
}

void Visualizer::updateGrid() {
    patches.clear();

    const auto original = octree.visualize();
    for (const auto &element: original) {
        if (element.domain.min[sliceAxis] > sliceValue || element.domain.max[sliceAxis] < sliceValue) continue;
        patches.push_back({element.domain, element.density});
    }

    slice = mesh.slice(sliceAxis, sliceValue);
}

void Visualizer::draw() {
    bool sceneChanged = false;
    if (req_scene != scene) {
        unloadScene();
        loadScene(req_scene);
        scene = req_scene;
        sceneChanged = true;
    }

    if (ImGui::Begin("Visualizer")) {
        drawGrid();
    }
    ImGui::End();

    if (ImGui::Begin("Controls")) {
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);

        static const char* current_scene = SceneNames[0];
        if (ImGui::BeginCombo("##scene", current_scene)) {
            for (int n = 0; n < IM_ARRAYSIZE(SceneNames); n++) {
                bool is_selected = (current_scene == SceneNames[n]); // You can store your selection however you want, outside or inside your objects
                if (ImGui::Selectable(SceneNames[n], is_selected)) {
                    current_scene = SceneNames[n];
                    req_scene = (size_t)n;
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
            }
            ImGui::EndCombo();
        }

        ImGui::PopItemWidth();

        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.50f);
        //ImGui::Text("%lu points in %lu polygons", slice.points.size(), slice.polygons.size());
        ImGui::DragFloat("Exposure", &exposure, 0.01f, -40, +40);
        ImGui::DragFloat("Outline opacity", &gridOutlineOpacity, 0.001f, 0, 1);
        ImGui::DragFloat("Dynamic range", &dynamicRange, 0.01f, 1, 40);

        bool needsUpdate = false;

        static const char *axisNames[] = { "X", "Y", "Z" };
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.20f);
        needsUpdate |= ImGui::Combo("##splitAxis", &sliceAxis, axisNames, 3, 3);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.50f);
        needsUpdate |= ImGui::DragFloat(
            "Slicing plane", &sliceValue,
            octree.aabb().diagonal()[sliceAxis] / 1000,
            octree.aabb().min[sliceAxis],
            octree.aabb().max[sliceAxis]
        );
        ImGui::PopItemWidth();

        if (sceneChanged || needsUpdate) {
            updateGrid();
        }

#ifndef __EMSCRIPTEN__
        if (ImGui::Button("Save SVG")) {
            saveSVG("density.svg");
        }
#endif

        ImGui::PopItemWidth();
        ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "Drag to pan, scroll to zoom");
    }
    ImGui::End();
}
