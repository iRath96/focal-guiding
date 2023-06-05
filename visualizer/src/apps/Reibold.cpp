#include "Reibold.hpp"
#include "imgui.h"
#include "../viridis.hpp"

#include <cmath>

struct SamplingDensity {
    virtual Env2D::Vector sample(float x0, PRNG &prng) const = 0;
    [[nodiscard]] virtual float pdf(float x0, const Env2D::Vector &direction) const = 0;
};

struct UniformSampling : public SamplingDensity {
    Env2D::Vector sample(float x0, PRNG &prng) const override {
        const float angle = float(M_PI) * (prng() - 0.5f);
        return { std::sin(angle), std::cos(angle) };
    }

    [[nodiscard]] float pdf(float x0, const Env2D::Vector &direction) const override {
        return float(1 / M_PI);
    }
};

struct FocalGuiding : public SamplingDensity {
    focal_guiding::Orthtree<Env2D> density;

    FocalGuiding() {
        Env2D::AABB aabb;
        aabb.min = { -1, -0.5f };
        aabb.max = { +1, 1.5f };
        density.setAABB(aabb);
    }

    Env2D::Vector sample(float x0, PRNG &prng) const override {
        return density.sample({ x0, 0 }, prng);
    }

    [[nodiscard]] float pdf(float x0, const Env2D::Vector &direction) const override {
        return density.pdf({ x0, 0 }, direction);
    }
};

struct ReiboldGuiding : public SamplingDensity {
    struct Configuration {
        int Knn = 10;
    } configuration;

    struct GuidePath {
        Reibold::Path path;

        float weight = 1; // *not* normalized to 1
        float mu0 = 0, mu1 = 0;
        float s00 = 0, s01 = 0, s11 = 0;

        [[nodiscard]] float pdfX0(float x0) const {
            return Gaussian(mu0, s00).pdf(x0);
        }

        [[nodiscard]] Gaussian conditional(float x0) const {
            // computes the conditional density for x1 given x0
            return {
                mu1 + s01 / s00 * (x0 - mu0),
                s11 - s01 / s00 * s01
            };
        }
    };
    std::vector<GuidePath> guides;

    [[nodiscard]] bool hasGuides() const { return !guides.empty(); }

    GuidePath sampleGuidePath(float x0, PRNG &prng) const {
        // weighted reservoir sampling to find guide path
        GuidePath selected{};
        float sum = 0;
        for (auto &guide : guides) {
            const float pdf = guide.weight * guide.pdfX0(x0);
            sum += pdf;
            if (prng() * sum < pdf) {
                selected = guide;
            }
        }
        assert(sum > 0);
        return selected;
    }

    [[nodiscard]] float probabilityGuidePath(float x0, const GuidePath &guidePath) const {
        // guidePath must come from this->guides for this to make sense
        float sum = 0;
        for (auto &alternative : guides) {
            sum += alternative.weight * alternative.pdfX0(x0);
        }
        return (guidePath.weight * guidePath.pdfX0(x0)) / sum;
    }

    Env2D::Vector sample(float x0, PRNG &prng) const override {
        const GuidePath guide = sampleGuidePath(x0, prng);
        const float x1 = guide.conditional(x0).sample(prng);
        if (std::isnan(x1)) {
            printf("nan\n");
            return {};
        }
        return Env2D::Vector { x1 - x0, 1 }.normalized();
    }

    [[nodiscard]] float pdf(float x0, const Env2D::Vector &direction) const override {
        const float distance = 1 / direction.y();
        const float cosine = direction.y();
        const float x1 = x0 + direction.x() * distance;

        // find normalization for selection probabilities of guide paths
        // (note: we do not use probabilityGuidePath below as that would cause O(N^2) complexity)
        float selectionNorm = 0;
        for (auto &guide : guides) {
            selectionNorm += guide.weight * guide.pdfX0(x0);
        }
        selectionNorm = 1 / selectionNorm;

        // compute pdf for x1 in area measure
        float pdfX1 = 0;
        for (auto &guide : guides) {
            const float guidePdfX1 = guide.conditional(x0).pdf(x1);
            const float guideSelProb = guide.weight * guide.pdfX0(x0) * selectionNorm;
            pdfX1 += guideSelProb * guidePdfX1;
        }

        // convert to angle measure
        return pdfX1 * distance / cosine;
    }

    static float pathDistance(const Reibold::Path &a, const Reibold::Path &b) {
        //return std::abs(b.x0 - a.x0) + std::abs(b.x1 - a.x1); // L1 test
        return std::pow(b.x0 - a.x0, 2.f) + std::pow(b.x1 - a.x1, 2.f);
    }

    struct KnnFinder {
        const std::vector<Reibold::Path> &paths;
        KnnFinder(const std::vector<Reibold::Path> &paths)
        : paths(paths) {
            refs.resize(paths.size());
        }

        void find(const Reibold::Path &path, int k, std::vector<const Reibold::Path *> &result) {
            assert(paths.size() >= k);
            result.clear();
            result.reserve(k);
            refs.clear();
            refs.reserve(paths.size());
            for (int i = 0; i < paths.size(); i++) {
                PathRef ref{};
                ref.index = i;
                ref.distance = pathDistance(path, paths[i]);
                refs.emplace_back(ref);
            }
            std::nth_element(refs.begin(), refs.begin() + k, refs.end());

            for (int i = 0; i < k; i++) {
                result.emplace_back(&paths[refs[i].index]);
            }
        }

    private:
        struct PathRef {
            int index;
            float distance;
            bool operator<(const PathRef &other) const { return distance < other.distance; }
        };
        std::vector<PathRef> refs;
    };

    void buildGuides(const std::vector<Reibold::Path> &paths) {
        guides.clear();
        if (paths.size() < configuration.Knn) return;

        KnnFinder knn { paths };
        std::vector<const Reibold::Path *> neighbors;
        for (auto &path : paths) {
            GuidePath guide{};
            guide.path = path;

            knn.find(path, configuration.Knn, neighbors);
            for (auto &neighbor : neighbors) {
                guide.mu0 += neighbor->x0;
                guide.mu1 += neighbor->x1;
                guide.s00 += neighbor->x0 * neighbor->x0;
                guide.s01 += neighbor->x0 * neighbor->x1;
                guide.s11 += neighbor->x1 * neighbor->x1;
                //printf("  neighbor %e to %e\n", neighbor->x0, neighbor->x1);
            }

            const float norm = 1 / float(neighbors.size());
            guide.mu0 *= norm;
            guide.mu1 *= norm;
            guide.s00 = norm * guide.s00 - guide.mu0 * guide.mu0;
            guide.s01 = norm * guide.s01 - guide.mu0 * guide.mu1;
            guide.s11 = norm * guide.s11 - guide.mu1 * guide.mu1;

            guide.weight = path.contribution;
            guides.emplace_back(guide);

            /*printf("guide w=%e %e to %e, kernel %e[%e] to %e[%e]\n",
                   guide.weight,
                   guide.path.x0, guide.path.x1,
                   guide.mu0, guide.s00,
                   guide.mu1, guide.s11);*/
        }
    }

    Reibold::ExamplePath examplePath(PRNG &prng, float minX0, float maxX0) const {
        const float x0 = minX0 + prng() * (maxX0 - minX0);
        const auto guide = sampleGuidePath(x0, prng);

        printf("guide w=%e %e to %e, kernel %e[%e] to %e[%e]\n",
               guide.weight,
               guide.path.x0, guide.path.x1,
               guide.mu0, guide.s00,
               guide.mu1, guide.s11);

        Reibold::ExamplePath result;
        result.guidePath = guide.path;
        result.x0 = Gaussian(guide.mu0, guide.s00);
        result.x1 = Gaussian(guide.mu1, guide.s11);
        result.conditionalX1 = guide.conditional(x0);
        result.samplePath.x0 = x0;
        result.samplePath.x1 = result.conditionalX1.sample(prng);
        result.contribution = 1 / pdf(x0, Env2D::Vector { result.samplePath.x1 - result.samplePath.x0, 1 }.normalized());

        result.guideNN.clear();
        if (guides.size() >= configuration.Knn) {
            result.guideNN.reserve(guides.size());
            for (auto &neighbor: guides) {
                result.guideNN.emplace_back(neighbor.path, pathDistance(guide.path, neighbor.path));
            }
            std::nth_element(result.guideNN.begin(), result.guideNN.begin() + configuration.Knn, result.guideNN.end());
            result.guideNN.resize(configuration.Knn);
        }

        return result;
    }
};

struct MixtureDensity : public SamplingDensity {
    SamplingDensity *a{};
    SamplingDensity *b{};
    float probB{};

    Env2D::Vector sample(float x0, PRNG &prng) const override {
        return (prng() < probB ? b : a)->sample(x0, prng);
    }

    [[nodiscard]] float pdf(float x0, const Env2D::Vector &direction) const override {
        if (probB <= 0) return a->pdf(x0, direction);
        if (probB >= 1) return b->pdf(x0, direction);
        return (1 - probB) * a->pdf(x0, direction) + probB * b->pdf(x0, direction);
    }
};

struct Sampling {
    UniformSampling uniform;
    FocalGuiding focal;
    ReiboldGuiding reibold;
    MixtureDensity mixture;

    void useFocalGuiding(float guidingProbability) {
        mixture.a = &uniform;
        mixture.b = &focal;
        mixture.probB = guidingProbability;
    }

    void useReiboldGuiding(float guidingProbability) {
        mixture.a = &uniform;
        mixture.b = &reibold;
        mixture.probB = guidingProbability;
    }

    void useUniformSampling() {
        mixture.a = &uniform;
        mixture.b = &uniform;
        mixture.probB = 0;
    }

    void setupFocalGuiding() {
        focal.density.configuration.minDepth = 4;
        focal.density.configuration.maxDepth = 12;
        focal.density.clear();
    }

    void setupReiboldGuiding() {
        reibold.guides.clear();
    }
};

void app() {
    static Reibold app;
    app.draw();
}

Reibold::Reibold() {
    image.resize(16);
    sampling = new Sampling;
}

void Reibold::sampleRay() {
    Path path{};
    path.x0 = 2 * prng() - 1;

    const auto direction = sampling->mixture.sample(path.x0, prng);
    path.x1 = path.x0 + direction.x() / direction.y();

    if (isPathBlocked(path.x0, path.x1)) {
        return;
    }

    const float f = 1;
    const float pdf = sampling->mixture.pdf(path.x0, direction);
    path.contribution = f / pdf;
    if (!(path.contribution > 0)) {
        printf("invalid contribution %e [f=%e, p=%e]\n", path.contribution, f, pdf);
    }

    splatImage(path.x0, path.contribution);
    sampling->focal.density.splat({ path.x0, 0 }, direction, 1 / direction.y(), path.contribution, pdf, prng);

    paths.emplace_back(path);
}

void Reibold::drawScene() {
    ImVec2 scale = ImGui::GetContentRegionAvail();
    scale.x /= 2;
    scale.y /= 2;
    scale.x = scale.y = scale.x;
    scale.y *= -1;

    ImVec2 shift = ImGui::GetCursorScreenPos();
    shift.x -= -1 * scale.x;
    shift.y -= 1.1f * scale.y;

    const auto transform = [&](float x, float y) {
        return ImVec2(x * scale.x + shift.x, y * scale.y + shift.y);
    };

    ImColor color;
    auto &drawList = *ImGui::GetWindowDrawList();

    // draw density
    if (setup.showDensity) {
        const float alphaNorm = pow(float(2), setup.exposure);
        const auto patches = sampling->focal.density.visualize();
        for (const auto &element: patches) {
            const auto v = turbo((log2(alphaNorm * element.density + 1.0f / 32) + 5) / 10.f);
            const ImColor pathColor{v.r, v.g, v.b, 1.0f};
            //const ImColor color = ImColor(1.f, 1.f, 1.f, alphaNorm * element.density);
            drawList.AddRectFilled(
                transform(element.domain.min.x(), element.domain.min.y()),
                transform(element.domain.max.x(), element.domain.max.y()),
                pathColor
            );
        }
    }

    // draw paths
    if (setup.showPaths) {
        color = ImColor(0.4f, 0.6f, 1.f, 0.2f);
        const float alphaNorm = pow(float(2), setup.pathExposure);
        for (const auto &path: paths) {
            color = ImColor(1.f, 1.f, 1.f, std::min(path.contribution * alphaNorm, 1.0f));
            if (path.x0 < setup.showMinX) continue;
            if (path.x0 > setup.showMaxX) continue;
            drawList.AddLine(transform(path.x0, 0), transform(path.x1, 1), color);
            //drawList.AddCircle(transform(path.mu0, 0), scale.x*std::sqrt(path.s00), color);
            /*drawList.AddLine(
                transform(path.x0 - 0.1f * std::sqrt(path.s00), 0.01f),
                transform(path.x0 + 0.1f * std::sqrt(path.s00), 0.01f),
                color
                );*/
        }
    }

    // draw walls
    color = IM_COL32_WHITE;
    drawList.AddLine(transform(-1, 1), transform(+1, 1), color);
    drawList.AddLine(transform(-1, 0), transform(+1, 0), color);

    // draw holes
    {
        float x = -(float(scene.numHoles - 1) * scene.holeDistance + scene.holeWidth) / 2;
        drawList.AddLine(transform(-1, 0.5f), transform(x, 0.5f), color);
        for (int i = 1; i < scene.numHoles; i++) {
            const float px = x;
            x += scene.holeDistance;
            drawList.AddLine(transform(px + scene.holeWidth, 0.5f), transform(x, 0.5f), color);
        }
        drawList.AddLine(transform(x + scene.holeWidth, 0.5f), transform(+1, 0.5f), color);
    }

    const auto drawGaussian = [&](const Gaussian &gaussian, float y, ImColor color) {
        drawList.AddCircle(transform(gaussian.mu, y), scale.x * std::sqrt(gaussian.s), color);
    };

    // draw example path
    if (setup.showExamplePath) {
        const ImColor green{0.f, 1.f, 0.f};
        const ImColor yellow{1.f, 1.f, 0.f};
        const ImColor pink{1.f, 0.f, 1.f};
        const auto &guide = examplePath.guidePath;
        const auto &sample = examplePath.samplePath;
        for (auto &nn : examplePath.guideNN) {
            drawList.AddLine(transform(nn.path.x0, 0), transform(nn.path.x1, 1), pink);
        }
        drawList.AddLine(transform(guide.x0, 0), transform(guide.x1, 1), green);
        drawList.AddLine(transform(sample.x0, 0), transform(sample.x1, 1), yellow);
        drawGaussian(examplePath.x0, 0, green);
        drawGaussian(examplePath.x1, 1, green);
        drawGaussian(examplePath.conditionalX1, 1, yellow);
        ImGui::TextColored(pink, "Example Contribution: %.3e\n", examplePath.contribution);
        ImGui::TextColored(green, "The green circle marks the unconditional gaussian (one stddev),");
        ImGui::SameLine();
        ImGui::TextColored(yellow, "the yellow circle marks the conditional gaussian (one stddev).");
    }

    // draw image
    if (true) {
        const float alphaNorm = 100;
        for (int x = 0; x < image.size(); x++) {
            const float x0 = float(2 * x) / float(image.size()) - 1;
            const float x1 = float(2 * (x + 1)) / float(image.size()) - 1;
            const auto &pixel = image[x];

            const auto v = turbo((log2(alphaNorm * pixel.mu + 1.0f / 32) + 5) / 10.f);
            const ImColor pixelColor{v.r, v.g, v.b, 1.0f};
            //const ImColor color = ImColor(1.f, 1.f, 1.f, alphaNorm * element.density);
            drawList.AddRectFilled(
                transform(x0, -0.01f),
                transform(x1, -0.11f),
                pixelColor
            );
        }
    }
}

void Reibold::iterationReibold() {
    paths.clear();
    clearImage();
    sampling->useReiboldGuiding(sampling->reibold.hasGuides() ? 0.5f : 0);

    const auto sizeBefore = paths.size();
    for (int i = 0; i < setup.numRays; i++) {
        sampleRay();
    }
    sampling->reibold.buildGuides(paths);
    iterationLog.push_back(int(paths.size() - sizeBefore));
}

void Reibold::sampleExamplePath() {
    examplePath = sampling->reibold.examplePath(prng, setup.showMinX, setup.showMaxX);
    if (isPathBlocked(examplePath.samplePath.x0, examplePath.samplePath.x1)) {
        examplePath.contribution = 0;
    }
    setup.showExamplePath = true;
}

void Reibold::saveSVG() {
    const float scale = 200;

    std::ofstream file{"frame.svg"};
    file << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
         << "width=\"512\" height=\"512\" "
         << "style=\"background: rgb(48, 18, 59);\" "
         << "viewBox=\"" << -scale << " " << 0 << " " << 2 * scale << " " << scale << "\">" << std::endl;

    // draw density
    if (setup.showDensity) {
        for (int step = 0; step < 2; step++) {
            file << "  <g style=\"stroke-opacity: 0.2;\">" << std::endl;
            const float alphaNorm = pow(float(2), setup.exposure);
            const auto patches = sampling->focal.density.visualize();
            for (const auto &element: patches) {
                const auto v = turbo((log2(alphaNorm * element.density + 1.0f / 32) + 5) / 10.f);
                const ImColor pathColor{v.r, v.g, v.b, 1.0f};
                //const ImColor color = ImColor(1.f, 1.f, 1.f, alphaNorm * element.density);
                file << "    <rect "
                     << "x=\"" << scale * element.domain.min.x()
                     << "\" y=\"" << scale * element.domain.min.y() << "\" "
                     << "width=\"" << scale * element.domain.diagonal().x()
                     << "\" height=\"" << scale * element.domain.diagonal().y() << "\" ";

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
    }

    // draw paths
    if (setup.showPaths) {
        file << "  <g>" << std::endl;

        const float alphaNorm = pow(float(2), setup.pathExposure);
        for (const auto &path: paths) {
            const float alpha = std::min(path.contribution * alphaNorm, 1.0f);
            file
                << "    <path stroke=\"white\" stroke-opacity=\"" << alpha << "\" fill=\"transparent\" d=\""
                << "M " << scale * path.x0 << " 0 "
                << "L " << scale * path.x1 << " " << scale
                << "\" />" << std::endl;
        }

        file << "  </g>" << std::endl;
    }

    file << "  <g>" << std::endl;
    file << "    <path stroke=\"white\" fill=\"transparent\" d=\"M "
        << -scale << " " << scale << " L " << scale << " " << scale << "\" />";
    file << "    <path stroke=\"white\" fill=\"transparent\" d=\"M "
        << -scale << " 0 L " << scale << " 0\" />";

    // draw holes
    {
        float x = -(float(scene.numHoles - 1) * scene.holeDistance + scene.holeWidth) / 2;
        file << "    <path stroke=\"white\" fill=\"transparent\" d=\"M "
            << -scale << " " << scale / 2 << " L "
            << scale * x << " " << scale / 2 << "\" />";
        for (int i = 1; i < scene.numHoles; i++) {
            const float px = x;
            x += scene.holeDistance;
            file
                << "    <path stroke=\"white\" fill=\"transparent\" d=\"M "
                << scale * (px + scene.holeWidth) << " " << scale / 2 << " L " << scale * x << " " << scale / 2 << "\" />";
        }
        file << "    <path stroke=\"white\" fill=\"transparent\" d=\"M "
            << scale * (x + scene.holeWidth) << " " << scale / 2 << " L " << scale << " " << scale / 2 << "\" />";
    }

    file << "  </g>" << std::endl;

    const auto drawGaussian = [&](const Gaussian &gaussian, float y, ImColor color) {
        auto c = color.Value;
        file
            << "    <circle stroke=\"rgb(" << int(255 * c.x) << " " << int(255 * c.y) << " " << int(255 * c.z) << ")\" "
            << "fill=\"transparent\" "
            << "cx=\"" << scale * gaussian.mu << "\" "
            << "cy=\"" << scale * y << "\" "
            << "r=\"" << scale * std::sqrt(gaussian.s) << "\" "
            << "/>";
    };

    const auto drawLine = [&](float x0, float y0, float x1, float y1, ImColor color) {
        auto c = color.Value;
        file << "    <path stroke=\"rgb(" << int(255 * c.x) << " " << int(255 * c.y) << " " << int(255 * c.z) << ")\" "
             << "fill=\"transparent\" d=\"M "
             << scale * x0 << " " << scale * y0 << " L "
             << scale * x1 << " " << scale * y1 << "\" />";
    };

    // draw example path
    if (setup.showExamplePath) {
        const ImColor green{0.f, 1.f, 0.f};
        const ImColor yellow{1.f, 1.f, 0.f};
        const ImColor pink{1.f, 0.f, 1.f};
        const auto &guide = examplePath.guidePath;
        const auto &sample = examplePath.samplePath;
        for (auto &nn : examplePath.guideNN) {
            drawLine(nn.path.x0, 0, nn.path.x1, 1, pink);
        }
        drawLine(guide.x0, 0, guide.x1, 1, green);
        drawLine(sample.x0, 0, sample.x1, 1, yellow);
        drawGaussian(examplePath.x0, 0, green);
        drawGaussian(examplePath.x1, 1, green);
        drawGaussian(examplePath.conditionalX1, 1, yellow);
        ImGui::TextColored(yellow, "Example Contribution: %.3e\n", examplePath.contribution);
    }

    // draw image
    /*if (true) {
        const float alphaNorm = 100;
        for (int x = 0; x < image.size(); x++) {
            const float x0 = float(2 * x) / float(image.size()) - 1;
            const float x1 = float(2 * (x + 1)) / float(image.size()) - 1;
            const auto &pixel = image[x];

            const auto v = turbo((log2(alphaNorm * pixel.mu + 1.0f / 32) + 5) / 10.f);
            const ImColor pixelColor{v.r, v.g, v.b, 1.0f};
            //const ImColor color = ImColor(1.f, 1.f, 1.f, alphaNorm * element.density);
            drawList.AddRectFilled(
                transform(x0, -0.01f),
                transform(x1, -0.11f),
                pixelColor
            );
        }
    }*/

    file << "</svg>" << std::endl;
}

void Reibold::draw() {
    //if (ImGui::BeginMainMenuBar()) {
    //    if (ImGui::BeginMenu("Demos")) {
    //        ImGui::MenuItem("ImGui demo", nullptr, &showImguiDemo);
    //        ImGui::EndMenu();
    //    }
    //
    //    ImGui::EndMainMenuBar();
    //}

    if (showImguiDemo) ImGui::ShowDemoWindow(&showImguiDemo);

    if (ImGui::Begin("Viewport")) {
        ImGui::Text("%ld paths", paths.size());
        drawScene();
    }
    ImGui::End();

    if (ImGui::Begin("Controls")) {
        ImGui::SeparatorText("Scene Configuration");
        ImGui::DragInt("#Holes", &scene.numHoles, 0.1f, 1, 10);
        ImGui::DragFloat("Hole Width", &scene.holeWidth, 0.001f, 0, scene.holeDistance);
        ImGui::DragFloat("Hole Distance", &scene.holeDistance, 0.001f, scene.holeWidth, 1);
        auto resolution = int(image.size());
        if (ImGui::DragInt("Image Resolution", &resolution, float(resolution) * 0.1f, 1, 1024)) {
            image.resize(resolution);
        }

        ImGui::SeparatorText("Simulation Settings");
        ImGui::DragInt("Reibold Knn", &sampling->reibold.configuration.Knn, 0.1f, 2, 32);
        ImGui::InputInt("#Rays", &setup.numRays);
        ImGui::DragInt("#Iterations", &setup.numIterations, 0.1f, 1, 16);

        ImGui::SeparatorText("Visualization Settings");
        ImGui::Checkbox("Show Density", &setup.showDensity);
        ImGui::Checkbox("Show Example", &setup.showExamplePath);
        ImGui::Checkbox("Show Paths", &setup.showPaths);
        ImGui::DragFloat("Exposure", &setup.exposure, 0.01f, -10, +10);
        ImGui::DragFloat("Path Exposure", &setup.pathExposure, 0.01f, -10, +10);

        if (setup.showPaths) {
            ImGui::DragFloat("Path Min X", &setup.showMinX, 0.01f, -1, +1);
            ImGui::DragFloat("Path Max X", &setup.showMaxX, 0.01f, -1, +1);
        }

        ImGui::SeparatorText("Simulation");

        if (ImGui::Button("Unguided")) {
            setup.showExamplePath = false;
            sampling->useUniformSampling();
            iterationLog.clear();
            paths.clear();

            clearImage();
            for (int i = 0; i < setup.numRays; i++) {
                sampleRay();
            }
            finishImage(setup.numRays);
            iterationLog.push_back(int(paths.size()));
        }
        ImGui::SameLine();
        if (ImGui::Button("Reibold")) {
            setup.showExamplePath = false;
            iterationLog.clear();
            sampling->setupReiboldGuiding();
            for (int iter = 0; iter < setup.numIterations; iter++) {
                iterationReibold();
            }
            finishImage(setup.numRays);
        }
        if (sampling->reibold.hasGuides()) {
            ImGui::SameLine();
            if (ImGui::Button("Reibold (one iter)")) {
                iterationReibold();
                finishImage(setup.numRays);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Ours")) {
            setup.showExamplePath = false;
            iterationLog.clear();
            sampling->setupFocalGuiding();
            for (int iter = 0; iter < setup.numIterations; iter++) {
                paths.clear();
                clearImage();
                sampling->useFocalGuiding(iter > 0 ? 0.5f : 0);

                const bool useIterativeNarrowing = iter >= (setup.numIterations / 2);
                sampling->focal.density.configuration.splattingStrategy = useIterativeNarrowing ?
                    focal_guiding::Orthtree<Env2D>::SPLAT_RAY_WEIGHTED :
                    focal_guiding::Orthtree<Env2D>::SPLAT_RAY;

                const auto sizeBefore = paths.size();
                for (int i = 0; i < setup.numRays; i++) {
                    sampleRay();
                }
                sampling->focal.density.build();
                iterationLog.push_back(int(paths.size() - sizeBefore));
            }
            finishImage(setup.numRays);
        }

        if (sampling->reibold.hasGuides()) {
            if (ImGui::Button("Example Path")) {
                sampleExamplePath();
            }
            ImGui::SameLine();
            if (ImGui::Button("Find Outlier")) {
                ExamplePath worst;
                worst.contribution = 0;
                for (int i = 0; i < 1000; i++) {
                    sampleExamplePath();
                    if (examplePath.contribution > worst.contribution) {
                        worst = examplePath;
                    }
                }
                examplePath = worst;
            }
        }

#ifndef __EMSCRIPTEN__
        if (ImGui::Button("Save SVG")) {
            saveSVG();
        }
#endif

        //for (auto &log : iterationLog) {
        //    ImGui::Text("%d", log);
        //}

        ImGui::SeparatorText("Image Statistics");

        for (int x = 0; x < image.size(); x++) {
            const auto &pixel = image[x];
            ImGui::Text("pixel %3d\tE=%.3e\tV=%.3e", x, pixel.mu, pixel.s);
        }
    }
    ImGui::End();
}

bool Reibold::isPathBlocked(float x0, float x1) const {
    const float normShift = -(float(scene.numHoles - 1) * scene.holeDistance + scene.holeWidth) / 2;
    const float mid = (x0 + x1) / 2 - normShift;
    if (std::fmod(mid, scene.holeDistance) > scene.holeWidth) {
        return true;
    }
    const int holeNum = int(std::floor(mid / scene.holeDistance));
    if (holeNum < 0 || holeNum >= scene.numHoles) {
        return true;
    }
    return false;
}
