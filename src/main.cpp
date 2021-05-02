//
//  main.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include <iostream>

#include "shared/FS.h"
#include "shared/CSV.h"
#include "shared/Timing.h"
#include "shared/SolverUtil.h"

#include "ebfr/Rig.h"
#include "ebfr/BlendshapeSolver.h"

#include "Args.h"

void onVertexStep(int iter, RigPtr rig, const std::string &dir) {
    static MeshPtr temp;
    if (temp == nullptr)
        temp = MakeMesh(rig->neutral());

    const std::string path = dir + "/bs-" + std::to_string(iter) + "-";
    const std::string ext = ".obj";

    for (auto bs = 0; bs < rig->numBlendshapes(); bs++) {
        AddVertices(rig->blendshape(bs).mesh(), rig->neutral(), 1.0, temp);

        WriteMesh(path + std::to_string(bs) + ext, temp);
    }
}

void onWeightsStep(int iter, RigPtr rig, const std::string &dir) {
    static MeshPtr temp;
    if (temp == nullptr)
        temp = MakeMesh(rig->neutral());

    //TIMER_START(WritePoses);

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

    PoseCSV::Write(JoinPath(dir, "weights-" + std::to_string(iter) + ".csv"), rig->weights());

    //TIMER_END(WritePoses);
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

    const auto estWeights = targetRig->weights();
    targetRig->randomizeWeights();

    BlendshapeSolver solver;

    solver.setDebugPath(args.debugPath);
    solver.setVertexStepCallback(onVertexStep);
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

    if (!solver.solve()) {
        std::cerr << "Failed to generate rigging" << std::endl;
        return 1;
    }

    std::cout << "Writing Final Blendshapes..." << std::endl;

    for (auto i = 1; i < targetRig->numBlendshapes(); i++) {
        WriteMesh(JoinPath(args.outputPath, std::to_string(i - 1) + ".obj"), targetRig->blendshape(i).mesh());
    }

    std::cout << "Writing Final Poses..." << std::endl;

    for (auto pose = 0; pose < targetRig->numPoses(); pose++) {
        WriteMesh(JoinPath(args.outputPath, "pose-" + std::to_string(pose) + ".obj"), targetRig->generatePose(estWeights[pose]));
    }

    std::cout << "Writing Final Weights..." << std::endl;

    PoseCSV::Write(JoinPath(args.outputPath, "poses.csv"), targetRig->weights());

    return 0;
}
