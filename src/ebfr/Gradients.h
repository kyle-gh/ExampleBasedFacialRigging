//
//  Gradients.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef Gradients_hpp
#define Gradients_hpp

#include "../shared/Matrix.h"
#include "Rig.h"

#include <vector>

class Gradients {
public:
    std::vector<std::vector<Matrix3x3>> blendshapeM;
    std::vector<std::vector<Matrix3x3>> blendshapeMInv;

    std::vector<std::vector<Matrix3x3>> blendshapeG;

    std::vector<std::vector<Matrix3x3>> poseM;

    void calculate(RigPtr rig, bool isTarget);
};

typedef std::shared_ptr<Gradients> GradientsPtr;

#endif /* Gradients_hpp */
