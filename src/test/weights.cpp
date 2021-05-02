//
//  weights.cpp
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

void onWeightsStep(int iter, RigPtr rig, const std::string &dir) {
    static MeshPtr temp;
    if (temp == nullptr)
        temp = MakeMesh(rig->neutral());

    std::cout
            << std::endl
            << "Writing Poses [" << iter << "]"
            << std::endl;

    TIMER_START(WritePoses);

    const std::string path = dir + "/pose-" + std::to_string(iter) + "-";
    const std::string ext = ".obj";

    for (auto pose = 0; pose < rig->numPoses(); pose++) {
        const auto &weights = rig->weights(pose);

        CopyVertices(temp, rig->neutral());

        for (auto bs = 0; bs < rig->numBlendshapes(); bs++) {
            AddVertices(temp, rig->blendshape(bs).mesh(), weights[bs]);
        }

        WriteMesh(path + std::to_string(pose) + ext, temp);
    }

    std::ofstream file;
    file.open(dir + "/weights-" + std::to_string(iter) + ".csv");

    for (auto pose = 0; pose < rig->numPoses(); pose++) {
        const auto &weights = rig->weights(pose);

        file << pose;

        for (auto w : weights)
            file << ", " << w;

        file << std::endl;
    }

    file.close();

    TIMER_END(WritePoses);
}

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

    // Use source weights as the estimated weights for target
//    for (auto i = 0; i < targetRig->numPoses(); i++)
//    {
//        auto& targetPose = targetRig->pose(i);
//        auto& sourcePose = sourceRig->pose(i);
//
//        std::copy(sourcePose.weights().begin(), sourcePose.weights().end(), std::back_inserter(targetPose.weights()));
//    }

    BlendshapeSolver solver;

    solver.setDebugPath(args.debugPath);
    solver.setWeightsStepCallback(onWeightsStep);

    solver.setMultithreaded(true);

    if (!solver.setSource(sourceRig)) {
        std::cerr << "Failed to set source" << std::endl;
        return 1;
    }

    if (!solver.setTarget(targetRig)) {
        std::cerr << "Failed to set target" << std::endl;
        return 1;
    }

    for (auto i = 0; i < sourceRig->numPoses(); i++) {
        targetRig->pose(i).setMesh(sourceRig->pose(i).mesh());

        auto &srcWeights = sourceRig->weights(i);
        auto &targetWeights = targetRig->weights(i);

        for (auto j = 1; j < srcWeights.size(); j++) {
            targetWeights[j] = srcWeights[j] * 0.2;
        }
    }

    for (auto i = 0; i < sourceRig->numBlendshapes(); i++) {
        targetRig->blendshape(i).setMesh(sourceRig->blendshape(i).mesh());
    }

    solver.testWeights(0);

    solver.testWeights(1);

    for (auto i = 0; i < sourceRig->numPoses(); i++) {
        auto &srcWeights = sourceRig->weights(i);
        auto &targetWeights = targetRig->weights(i);

        std::cout << "Pose: " << i << std::endl;

        for (auto j = 0; j < srcWeights.size(); j++) {
            std::cout << srcWeights[j] << " -> " << targetWeights[j] << std::endl;
        }

        std::cout << std::endl;
        break;
    }

    return 0;
}
