//
//  posegen.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <random>
#include <filesystem>

#include "../shared/Timing.h"
#include "../shared/SolverUtil.h"

#include "../ebfr/Rig.h"

#include <cxxopts.hpp>

int main(int argc, char *argv[]) {
    cxxopts::Options options("posegen", "Generate poses from blendshapes");

    options.add_options()
            ("blendshapes", "Path to the source reference mesh", cxxopts::value<std::vector<std::string>>())
            ("num", "Path to the source reference mesh", cxxopts::value<int>())
            ("output", "Path to save the deformed target mesh to", cxxopts::value<std::string>());

    std::vector<std::string> blendshapeDirs;
    std::string outputPath;
    int numPoses;

    try {
        auto result = options.parse(argc, argv);

        if (!result.count("blendshapes") || !result.count("num") || !result.count("output")) {
            std::cout << options.help() << std::endl;
            exit(1);
        }

        blendshapeDirs = result["blendshapes"].as<std::vector<std::string>>();
        numPoses = result["num"].as<int>();
        outputPath = result["output"].as<std::string>();
    }
    catch (const cxxopts::OptionException &e) {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::vector<std::vector<std::vector<int>>> groups = {
            {{27, 28}, {0, 1}}, // Eyes
            {{24, 25}}, // Nose
            {{22}, {23}, {17}, {16}}, // Cheeks/Mouth
            {{20, 21}, {18, 19}, {14, 15}, {13}, {12}, {11}, {10}, {9}, {4, 5}}, // Mouths
            {{9}, {7, 8}, {6}}, // Lower Jaw/Mouth
            {{2, 3}} // Brows
    };

    std::vector<std::vector<int>> remaining;

    remaining.resize(groups.size());
    for (auto i = 0; i < groups.size(); i++) {
        const auto num = groups[i].size();

        auto &list = remaining[i];

        list.resize(num);
        for (auto j = 0; j < num; j++) {
            list[j] = j;
        }

        std::shuffle(list.begin(), list.end(), gen);
    }

    std::normal_distribution<> n{0.6, 0.4};

    std::vector<RigPtr> rigs;
    for (const auto &blendshapeDir : blendshapeDirs) {
        std::cout << rigs.size() << ": " << blendshapeDir << std::endl;

        std::filesystem::create_directories(outputPath + "/" + std::to_string(rigs.size()));

        auto rig = MakeRig();

        rig->loadBlendshapes(blendshapeDir);

        rigs.push_back(rig);
    }

    std::vector<double> weights(rigs[0]->numBlendshapes() - 1);

    std::ofstream poseWeights;
    poseWeights.open(outputPath + "/random.csv", std::ofstream::out | std::ofstream::app);

    poseWeights << "Pose,";
    for (auto i = 0; i < weights.size(); i++) {
        poseWeights << i << ",";
    }
    poseWeights << std::endl;

    for (auto i = 0; i < numPoses; i++) {
        for (auto &w : weights) {
            w = 0.0;
        }

        for (auto j = 0; j < groups.size(); j++) {
            const auto &list = groups[j];

            int index = 0;

            if (remaining[j].empty()) {
                std::uniform_int_distribution<> r(-1, list.size() - 1);

                index = r(gen);
            } else {
                index = remaining[j].back();
                remaining[j].pop_back();
            }

            if (index == -1) {
                continue;
            }

            const auto &group = list[index];

            for (auto bs : group) {
                weights[bs] = std::clamp(n(gen), 0.25, 1.0);
            }
        }

        if (i != 0) {
            poseWeights << std::endl;
        }

        poseWeights << i << ",";

        for (auto w : weights) {
            poseWeights << w << ",";
        }

        for (auto r = 0; r < rigs.size(); r++) {
            auto pose = rigs[r]->generatePose(weights);

            WriteMesh(outputPath + "/" + std::to_string(r) + "/" + std::to_string(i) + ".obj", pose);
        }
    }

    return 0;
}
