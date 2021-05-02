//
//  BlendshapeSolver.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef Resolver_hpp
#define Resolver_hpp

#include <stdio.h>
#include <vector>

#include "../shared/Matrix.h"

#include "Rig.h"

#include "GradientSolver.h"
#include "VertexSolver.h"
#include "WeightsSolver.h"

class BlendshapeSolver {
public:
    BlendshapeSolver();

    void setRegularizationConsts(const ParameterD &k, const ParameterD &theta);

    void setBlendshapeSolveConsts(const ParameterD &beta);

    void setWeightSolveConsts(const ParameterD &lambda);

    void setNumIterations(int num);

    void setVertexStepCallback(VertexSolver::StepCallback callback);

    void setWeightsStepCallback(WeightsSolver::StepCallback callback);

    void setDebugPath(const std::string &path);

    void setMultithreaded(bool enable);

    bool setSource(RigPtr rig);

    bool setTarget(RigPtr rig);

    RigPtr getSource() const;

    GradientsPtr getSourceGradients() const;

    RigPtr getTarget() const;

    GradientsPtr getTargetGradients() const;

    bool solve();

    bool testGradient(int iter);

    bool testVertex(int iter, int bs);

    bool testWeights(int iter);

private:
    RigPtr _source;

    GradientsPtr _sourceGradients;

    RigPtr _target;

    GradientsPtr _targetGradients;

    int _numIterations;

    // Stage A - Solve for Blendshape Gradients
    GradientSolver _gradientSolver;

    // Stage T - Solve for Vertices, from Gradients
    VertexSolver _vertexSolver;

    // Stage B - Solve for Weights
    WeightsSolver _weightsSolver;

    void init();

    void initGradient();

    void initVertex();

    void initWeights();

    void rebuildBlendshapes();

    void rebuildGradients();

    void rebuildPoses();
};

#endif /* Resolver_hpp */
