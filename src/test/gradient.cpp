//
//  gradient.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include <iostream>
#include <fstream>

#include "../shared/Timing.h"
#include "../shared/SolverUtil.h"

#include "../ebfr/Rig.h"
#include "../ebfr/BlendshapeSolver.h"

#include "../Args.h"

int main(int argc, char *argv[]) {
    Args args;
    args.read(argc, argv);

    Eigen::initParallel();
    //Eigen::setNbThreads(4);

    TIMER_START(LoadSourceRig);

    auto sourceRig = MakeRig();
    sourceRig->load(args.srcBlendshapeDir, args.srcPoseDir, args.srcWeightsPath, args.vertexMaskPath, false);

    TIMER_END(LoadSourceRig);

    TIMER_START(LoadTargetRig);

    auto targetRig = MakeRig();
    targetRig->load(args.tgtNeutralPath, args.tgtPoseDir, args.tgtWeightsPath, args.vertexMaskPath, true);

    targetRig->generateEmptyBlendshapes(sourceRig->numBlendshapes());

    TIMER_END(LoadTargetRig);

    BlendshapeSolver solver;

    solver.setDebugPath(args.debugPath);

    solver.setMultithreaded(true);

    if (!solver.setSource(sourceRig)) {
        std::cerr << "Failed to set source" << std::endl;
        return 1;
    }

    if (!solver.setTarget(targetRig)) {
        std::cerr << "Failed to set target" << std::endl;
        return 1;
    }

    for (auto iter = 0; iter < 5; iter++) {
        const auto iterWeight = (iter + 1) / 5.0f;

        std::cout << "weight: " << iterWeight << std::endl;

        for (auto i = 0; i < sourceRig->numPoses(); i++) {
            auto &srcWeights = sourceRig->weights(i);
            auto &targetWeights = targetRig->weights(i);

            std::cout << "Pose " << i << ": ";

            for (auto j = 0; j < srcWeights.size(); j++) {
                targetWeights[j] = srcWeights[j] * iterWeight;
                std::cout << srcWeights[j] << ", ";
            }

            std::cout << std::endl;
        }

        solver.testGradient(iter);

        auto face = solver.getTarget()->face(0);

        std::cout << "Expected: " << std::endl << solver.getSourceGradients()->blendshapeM[1][face] << std::endl
                  << "- - -" << std::endl
                  << "Result: " << std::endl << solver.getTargetGradients()->blendshapeM[1][face] << std::endl
                  << std::endl << std::endl;

        std::cout << "Expected: " << std::endl << solver.getSourceGradients()->blendshapeM[2][face] << std::endl
                  << "- - -" << std::endl
                  << "Result: " << std::endl << solver.getTargetGradients()->blendshapeM[2][face] << std::endl
                  << std::endl;
    }

    return 0;
}
