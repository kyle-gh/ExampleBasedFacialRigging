//
//  BlendshapeSolver.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "BlendshapeSolver.h"

#include "../shared/SolverUtil.h"
#include "../shared/Timing.h"

BlendshapeSolver::BlendshapeSolver()
        : _numIterations(10) {
    setBlendshapeSolveConsts(ParameterD(ParameterD::Continuous, {{0,              0.5},
                                                                 {_numIterations, 0.1}}));
    setRegularizationConsts(ParameterD(0.1), ParameterD(2.0));
    setWeightSolveConsts(ParameterD(ParameterD::Continuous, {{0,              1000},
                                                             {_numIterations, 100}}));
}

void BlendshapeSolver::setNumIterations(int num) {
    _numIterations = num;
}

void BlendshapeSolver::setRegularizationConsts(const ParameterD &k, const ParameterD &theta) {
    _gradientSolver.setRegularizationConsts(k, theta);
}

void BlendshapeSolver::setBlendshapeSolveConsts(const ParameterD &beta) {
    _gradientSolver.setBlendshapeSolveConsts(beta);
}

void BlendshapeSolver::setWeightSolveConsts(const ParameterD &lambda) {
    _weightsSolver.setLambda(lambda);
}

void BlendshapeSolver::setVertexStepCallback(VertexSolver::StepCallback callback) {
    _vertexSolver.setStepCallback(callback);
}

void BlendshapeSolver::setWeightsStepCallback(WeightsSolver::StepCallback callback) {
    _weightsSolver.setStepCallback(callback);
}

void BlendshapeSolver::setDebugPath(const std::string &path) {
    _gradientSolver.setDebugPath(path);
    _vertexSolver.setDebugPath(path);
    _weightsSolver.setDebugPath(path);
}

void BlendshapeSolver::setMultithreaded(bool enable) {
    _gradientSolver.setMultithreaded(enable);
    _vertexSolver.setMultithreaded(enable);
    _weightsSolver.setMultithreaded(enable);
}

bool BlendshapeSolver::setSource(RigPtr rig) {
    _source = rig;

    _sourceGradients = std::make_shared<Gradients>();
    _sourceGradients->calculate(rig, false);

    return _source != nullptr;
}

bool BlendshapeSolver::setTarget(RigPtr rig) {
    _target = rig;

    _targetGradients = std::make_shared<Gradients>();
    _targetGradients->calculate(rig, true);

    return _target != nullptr;
}

RigPtr BlendshapeSolver::getSource() const {
    return _source;
}

GradientsPtr BlendshapeSolver::getSourceGradients() const {
    return _sourceGradients;
}

RigPtr BlendshapeSolver::getTarget() const {
    return _target;
}

GradientsPtr BlendshapeSolver::getTargetGradients() const {
    return _targetGradients;
}

bool BlendshapeSolver::solve() {
    TIMER_START(Solve)

    init();

    initGradient();

    initVertex();

    initWeights();

    for (auto i = 0; i < _numIterations; i++) {
        TIMER_START(Iteration)

        // Optimize Blendshapes

        TIMER_START(GradientSolve);

        // Estimates blendshapes gradients
        if (!_gradientSolver.solve(i))
            return false;

        TIMER_END(GradientSolve);

        TIMER_START(VertexSolve)

        // Calculates blendshapes vertices from gradients
        // See Deform. Transfer
        if (!_vertexSolver.solve(i))
            return false;

        TIMER_END(VertexSolve)

//        TIMER_START(RebuildGradients)
//        
//        rebuildGradients();
//        
//        TIMER_END(RebuildGradients);

//        rebuildBlendshapes();

        TIMER_START(WeightsSolver)

        // Estimates blendshape weights per pose
        if (!_weightsSolver.solve(i))
            return false;

        TIMER_END(WeightsSolver)

//        TIMER_START(RebuildPoses);
//
//        rebuildPoses();
//
//        TIMER_END(RebuildPoses);

        TIMER_END(Iteration)
    }

    TIMER_END(Solve)

    return true;
}

bool BlendshapeSolver::testGradient(int iter) {
    if (iter == 0) {
        init();

        initGradient();
    }

    TIMER_START(GradientSolve);

    if (!_gradientSolver.solve(iter))
        return false;

    TIMER_END(GradientSolve);

    return true;
}

bool BlendshapeSolver::testVertex(int iter, int bs) {
    if (iter == 0) {
        init();

        initVertex();
    } else {

        TIMER_START(VertexSolve);

        if (!_vertexSolver.solve(iter, bs))
            return false;

        TIMER_END(VertexSolve);
    }

    return true;
}

bool BlendshapeSolver::testWeights(int iter) {
    if (iter == 0) {
        init();

        initWeights();
    } else {
        TIMER_START(WeightsSolver)

        // Estimates blendshape weights per pose
        if (!_weightsSolver.solve(iter))
            return false;

        TIMER_END(WeightsSolver)
    }

    return true;
}

void BlendshapeSolver::init() {
}

void BlendshapeSolver::initGradient() {
    TIMER_START(GradientInit);

    _gradientSolver.setSource(_source, _sourceGradients);
    _gradientSolver.setTarget(_target, _targetGradients);
    _gradientSolver.init();

    TIMER_END(GradientInit);
}

void BlendshapeSolver::initVertex() {
    TIMER_START(VertexInit);

    _vertexSolver.setSource(_source, _sourceGradients);
    _vertexSolver.setTarget(_target, _targetGradients);
    _vertexSolver.init();

    TIMER_END(VertexInit);
}

void BlendshapeSolver::initWeights() {
    TIMER_START(WeightsInit);

    _weightsSolver.setSource(_source, _sourceGradients);
    _weightsSolver.setTarget(_target, _targetGradients);
    _weightsSolver.init();

    TIMER_END(WeightsInit);
}

void BlendshapeSolver::rebuildBlendshapes() {
    const auto neutral = _target->neutral();

    for (auto i = 0; i < _target->numBlendshapes(); i++) {
        auto blendshape = _target->blendshape(i).mesh();

        AddVertices(blendshape, neutral, -1, blendshape);
    }
}

void BlendshapeSolver::rebuildGradients() {
    for (auto i = 0; i < _target->numBlendshapes(); i++) {
        //AddVertices(_target->neutral(), _target->blendshape(i).mesh(), 1, temp);

        // M_b
        CalculateFrames(_target->blendshape(i).mesh(), _targetGradients->blendshapeM[i]);
    }
}

void BlendshapeSolver::rebuildPoses() {
    for (auto pose = 0; pose < _target->numPoses(); pose++) {
        const auto weights = _target->pose(pose).weights();

        auto poseMesh = _target->pose(pose).mesh();

        CopyVertices(poseMesh, _target->neutral());

        for (auto bs = 0; bs < _target->numBlendshapes(); bs++) {
            AddVertices(poseMesh, _target->blendshape(bs).mesh(), weights[bs]);
        }

        CalculateFrames(poseMesh, _targetGradients->poseM[pose]);
    }
}

