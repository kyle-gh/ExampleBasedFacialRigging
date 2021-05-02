//
//  SolverBase.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "SolverBase.h"

SolverBase::SolverBase()
: _iteration(0)
, _source(nullptr)
, _sourceGradients(nullptr)
, _target(nullptr)
, _targetGradients(nullptr)
, _useMultithreaded(false)
, _debugPath()
, _debug(false)
{
}

bool SolverBase::setSource(RigPtr rig, GradientsPtr gradients) {
    _source = rig;
    _sourceGradients = gradients;

    return _source != nullptr && _sourceGradients != nullptr;
}

bool SolverBase::setTarget(RigPtr rig, GradientsPtr gradients) {
    _target = rig;
    _targetGradients = gradients;

    return _target != nullptr && _targetGradients != nullptr;
}

void SolverBase::setMultithreaded(bool enable) {
    _useMultithreaded = enable;
}

bool SolverBase::checkSolverError(Eigen::ComputationInfo info) const {
    if (info == Eigen::Success)
        return true;

    std::cerr << "**Eigen Solver Error: " << Error(info) << std::endl;
    return false;
}

