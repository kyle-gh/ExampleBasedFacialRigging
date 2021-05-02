//
//  vertex.cpp
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

void onVertexStep(int iter, RigPtr rig, const std::string &dir) {
    static MeshPtr temp;
    if (temp == nullptr)
        temp = MakeMesh(rig->neutral());

    std::cout
            << std::endl
            << "Writing Blendshapes [" << iter << "]"
            << std::endl;

    TIMER_START(WriteBlendshapes);

    const std::string path = dir + "/bs-" + std::to_string(iter) + "-";
    const std::string ext = ".obj";

    for (auto bs = 0; bs < rig->numBlendshapes(); bs++) {
        AddVertices(rig->blendshape(bs).mesh(), rig->neutral(), 1.0, temp);

        WriteMesh(path + std::to_string(bs) + ext, temp);
    }

    TIMER_END(WriteBlendshapes);
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
    solver.setVertexStepCallback(onVertexStep);

    solver.setMultithreaded(true);

    if (!solver.setSource(sourceRig)) {
        std::cerr << "Failed to set source" << std::endl;
        return 1;
    }

    if (!solver.setTarget(targetRig)) {
        std::cerr << "Failed to set target" << std::endl;
        return 1;
    }

    auto target = solver.getTargetGradients();
    auto neutral = sourceRig->neutral();
    auto btemp = MakeMesh(neutral);

    for (auto i = 0; i < sourceRig->numBlendshapes(); i++) {
        const auto mesh = sourceRig->blendshape(i).mesh();
        auto &m = target->blendshapeM[i];

        if (i == 0) {
            CalculateFrames(mesh, m);
        } else {
            AddVertices(mesh, neutral, 1, btemp);

            CalculateFrames(btemp, m);

            const auto &neutralM = target->blendshapeM[0];

            for (auto j = 0; j < mesh->n_faces(); j++) {
                m[j] -= neutralM[j];
            }
        }
    }

    solver.testVertex(0, 0);

    auto bs = 5;

    solver.testVertex(10, bs);

    auto neutralPoints = solver.getSource()->blendshape(0).mesh()->points();
    auto points = solver.getSource()->blendshape(bs).mesh()->points();
    auto v = 0;
    v = solver.getSource()->vertex(0);
    std::cout << v << ": " << points[v] << " :: " << points[v] + neutralPoints[v] << std::endl;
    v = solver.getSource()->vertex(1);
    std::cout << v << ": " << points[v] << " :: " << points[v] + neutralPoints[v] << std::endl;
    v = solver.getSource()->vertex(2);
    std::cout << v << ": " << points[v] << " :: " << points[v] + neutralPoints[v] << std::endl;

    return 0;
}
