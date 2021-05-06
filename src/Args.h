//
//  Args.h
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef EXAMPLEBASEDFACIALRIGGING_ARGS_H
#define EXAMPLEBASEDFACIALRIGGING_ARGS_H

#include <cxxopts.hpp>

struct Args {
    std::string srcBlendshapeDir;
    std::string srcPoseDir;
    std::string srcWeightsPath;
    std::string tgtNeutralPath;
    std::string tgtPoseDir;
    std::string tgtWeightsPath;
    std::string vertexMaskPath;
    std::string outputPath;
    std::string debugPath;

    bool read(int argc, char *argv[]) {
        cxxopts::Options options("ebfr", "Generate a facial blendshape rig from example poses");

        options.add_options()
                ("source-blendshapes", "Path to the directory containing the source blendshape", cxxopts::value<std::string>())
                ("source-poses", "Path to the directory containing the source pose mesh files", cxxopts::value<std::string>())
                ("source-weights", "Path to the source pose-weights file", cxxopts::value<std::string>())

                ("target-neutral", "Path to the target neutral mesh file", cxxopts::value<std::string>())
                ("target-poses", "Path to the directory containing the target pose mesh files", cxxopts::value<std::string>())
                ("target-weights", "Path to the target (estimated) pose-weights file", cxxopts::value<std::string>())

                ("vertex-mask", "Path to the vertex mask file", cxxopts::value<std::string>())

                ("output", "Path to a directory to write the final target blendshapes", cxxopts::value<std::string>())
                ("debug", "Path to a directory to save in-progress data (meshes, weights, etc.", cxxopts::value<std::string>());

        try {
            auto result = options.parse(argc, argv);

            if (!result.count("source-blendshapes") || !result.count("source-poses") || !result.count("source-weights") ||
                !result.count("target-neutral") || !result.count("target-poses") || !result.count("target-weights") ||
                !result.count("output")
                ) {
                std::cout << options.help() << std::endl;
                exit(1);
            }

            srcBlendshapeDir = result["source-blendshapes"].as<std::string>();
            srcPoseDir = result["source-poses"].as<std::string>();
            srcWeightsPath = result["source-weights"].as<std::string>();
            tgtNeutralPath = result["target-neutral"].as<std::string>();
            tgtPoseDir = result["target-poses"].as<std::string>();
            tgtWeightsPath = result["target-weights"].as<std::string>();
            vertexMaskPath = result["vertex-mask"].as<std::string>();
            outputPath = result["output"].as<std::string>();

            if (result.count("debug")) {
                debugPath = result["debug"].as<std::string>();
            }
        }
        catch (const cxxopts::OptionException &e) {
            std::cout << "error parsing options: " << e.what() << std::endl;
            exit(1);
        }

        return true;
    }
};

#endif //EXAMPLEBASEDFACIALRIGGING_ARGS_H
