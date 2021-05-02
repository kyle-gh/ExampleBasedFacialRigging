//
//  SolverBase.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef SolverBase_hpp
#define SolverBase_hpp

#include <stdio.h>
#include <memory>

#include "../shared/Matrix.h"
#include "../shared/Mesh.h"

#include "Rig.h"
#include "Gradients.h"
#include "Parameter.h"

typedef unsigned int Index;

class SolverBase {
public:
    typedef std::function<void(int, RigPtr, const std::string &)> StepCallback;

    SolverBase();

    bool setSource(RigPtr rig, GradientsPtr gradients);

    bool setTarget(RigPtr rig, GradientsPtr gradients);

    void setMultithreaded(bool enable);

    void setDebugPath(const std::string &path) { _debugPath = path; }

    void setStepCallback(StepCallback callback) { _callback = callback; }

    virtual void init() {}

    virtual bool solve(int iter) {
        _iteration = iter;
        return true;
    }

protected:
    bool _debug;

    int _iteration;

    bool _useMultithreaded;

    RigPtr _source;
    GradientsPtr _sourceGradients;

    RigPtr _target;
    GradientsPtr _targetGradients;

    StepCallback _callback;

    std::string _debugPath;

    bool checkSolverError(Eigen::ComputationInfo info) const;
};

#endif /* SolverBase_hpp */
