//
//  Gradients.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "Gradients.h"

#include "../shared/SolverUtil.h"

void Gradients::calculate(RigPtr rig, bool isTarget) {
    const auto &poses = rig->poses();
    const auto &blendshapes = rig->blendshapes();

    //// Calculate Frames - M_(A_i)
    poseM.resize(poses.size());

    for (auto i = 0; i < poses.size(); i++)
        CalculateFrames(poses[i].mesh(), poseM[i]);

    blendshapeM.resize(blendshapes.size());

    auto neutral = rig->neutral();

    for (auto bs = 0; bs < rig->numBlendshapes(); bs++) {
        const auto mesh = rig->blendshape(bs).mesh();
        auto &m = blendshapeM[bs];

        if (bs == 0) {
            CalculateFrames(mesh, m);
        } else {
            if (isTarget) {
                GenerateEmptyFrames(neutral, m);
            } else {
                CalculateFrames(mesh, m);
            }
        }
    }

    blendshapeMInv.resize(1);
    blendshapeMInv[0].resize(blendshapeM[0].size());

    for (auto i = 0; i < blendshapeM[0].size(); i++) {
        blendshapeMInv[0][i] = blendshapeM[0][i].inverse();
    }
}