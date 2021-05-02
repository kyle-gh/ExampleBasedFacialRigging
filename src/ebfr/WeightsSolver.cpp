//
//  WeightsSolver.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "WeightsSolver.h"

#include <thread>
#include <mutex>

int WeightsSolver::WeightsFunctor::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
    const auto numVertices = a->rows() / 3;
    const auto numWeights = estimateW.size();

    // ||Ax - c||^2
    // where A are the blendshapes, x are the weights, and c is (pose - neutral)
    MatrixX vDiff = (*a * x).rowwise().sum() - *c;
    vDiff = vDiff.cwiseProduct(vDiff);

    for (auto i = 0; i < numVertices; i++) {
        const auto sum = vDiff.block<3, 1>(i * 3, 0).sum();
        fvec(i) = sum;//std::sqrt(sum);
    }

    // Regularization
    // ||x - w*||^2
    // where x are the weights and w* are the user-estimated weights
    VectorX wDiff = estimateW - x;
    wDiff = wDiff.cwiseProduct(wDiff);
    wDiff *= lambda;

    for (auto i = 0; i < numWeights; i++)
        fvec(i + numVertices) = wDiff(i);

    return 0;
}

int WeightsSolver::WeightsFunctor::inputs() const {
    return estimateW.size();
}

int WeightsSolver::WeightsFunctor::values() const {
    // #  of vertices + # of weights
    return (a->rows() / 3) + estimateW.size();
}

WeightsSolver::WeightsSolver()
: SolverBase()
, _lambda(1000) // -> 100
, _maxIterations(10)
, _minWeight(0.0)
, _maxWeight(1.0)
{

}

void WeightsSolver::setLambda(const ParameterD &lambda) {
    _lambda = lambda;
}

void WeightsSolver::init() {
    const auto rows = _target->numVertices() * Vector3::SizeAtCompileTime;
    const auto cols = _target->numBlendshapes() - 1;

    _a.resize(rows, cols);

    // Save the user-provided weight estimates and the precalculate the
    // "c" matrix (pose - neutral).
    _estimateWs.resize(_target->numPoses());
    _Cs.resize(_target->numPoses());

    for (auto pose = 0; pose < _target->numPoses(); pose++) {
        copyWeightsTo(_target->weights(pose), _estimateWs[pose]);

        _Cs[pose].resize(_target->numVertices() * Vector3::SizeAtCompileTime);

        appendWeightFit(pose, _Cs[pose]);
    }
}

bool WeightsSolver::solve(int iter) {
    std::cout
            << std::endl
            << "Weights Solver [" << iter << "]" << std::endl
            << "\tLambda: " << _lambda(iter) << std::endl
            << std::endl;

    SolverBase::solve(iter);

    std::mutex logMutex;

    // All of the problems share the same blendshapes, so
    // the "A" matrix can be shared with all problems.
    appendWeightFit(0, _a);

    auto solverOp =
            [this, &logMutex]
                    (int threadId, size_t poseStart, size_t poseEnd) {
                if (_debug && threadId >= 0) {
                    logMutex.lock();
                    std::cout << "Thread [" << threadId << "-WS] Starting: Poses " << poseStart << " -> " << poseEnd << " (" << (poseEnd - poseStart) << ")" << std::endl;
                    logMutex.unlock();
                }

                WeightsFunctorNumericalDiff data;

                initSolverData(data);

                //appendWeightFit(0, data._a);

                auto pose = (Index) poseStart;

                while (true) {
                    if (pose >= poseEnd)
                        break;

                    data.a = &_a;
                    data.c = &_Cs[pose];

                    data.estimateW = _estimateWs[pose];
                    data.x = _estimateWs[pose];

                    //copyWeightsTo(_target->weights(pose), data.x);

                    Eigen::LevenbergMarquardt<WeightsFunctorNumericalDiff> lm(data);

                    // auto status = lm.minimize(data.x);

                    // Use Eigen's (experimental) Levenberg-Marquardt implementation to
                    // optimize for the blendshape weights for each pose.
                    auto status = lm.minimizeInit(data.x);
                    if (status == Eigen::LevenbergMarquardtSpace::ImproperInputParameters) {
                        std::cerr << "Weights Solver failed to init" << std::endl;
                        continue;
                    }

                    auto iter = 0;
                    do {
                        status = lm.minimizeOneStep(data.x);

                        // Hack-ish box constraint
                        for (auto i = 0; i < data.x.size(); i++) {
                            data.x(i) = std::clamp(data.x(i), _minWeight, _maxWeight);
                        }

                        iter++;

                    } while (status == Eigen::LevenbergMarquardtSpace::Running && iter < _maxIterations);

                    copyWeightsTo(data.x, _target->weights(pose));

                    pose++;
                }

                if (_debug && threadId >= 0) {
                    logMutex.lock();
                    std::cout << "Thread [" << threadId << "-WS] Complete" << std::endl;
                    logMutex.unlock();
                }
            };

    if (_useMultithreaded) {
        std::vector<std::thread> pool;
        const auto size = _target->numPoses();
        const auto numThreads = std::min(size, (size_t) std::thread::hardware_concurrency());
        const auto segmentSize = numThreads == size ? 1 : (size / numThreads) + 1;
        auto segmentStart = 0;

        for (auto i = 0; i < numThreads; i++) {
            pool.push_back(std::thread(solverOp, i, segmentStart, std::min(segmentStart + segmentSize, size)));

            segmentStart += segmentSize;
        }

        for (auto &thread : pool) {
            thread.join();
        }
    } else {
        solverOp(-1, 0, _target->numPoses());
    }

    if (_callback != nullptr)
        _callback(iter, _target, _debugPath);

    return true;
}

void WeightsSolver::initSolverData(WeightsSolver::WeightsFunctor &data) {
    // const auto rows = (_target->numVertices() * Vector3::SizeAtCompileTime);
    // const auto cols = _target->numBlendshapes() - 1;

    //data._a.resize(rows, cols);

    //data.c.resize(rows);
    //data.vDiff.resize(rows);

    data.estimateW.resize(_target->numBlendshapes() - 1);
    //data.wDiff.resize(_target->numBlendshapes() - 1);

    data.lambda = _lambda(_iteration);
}

void WeightsSolver::appendWeightFit(Index pose, MatrixX &a, VectorX &c) {
    appendWeightFit(pose, a);

    appendWeightFit(pose, c);
}

void WeightsSolver::appendWeightFit(Index pose, MatrixX &a) {
    for (Index bs = 1; bs < _target->numBlendshapes(); bs++) {
        appendWeightFit(pose, bs, a);
    }
}

void WeightsSolver::appendWeightFit(Index pose, Index bs, MatrixX &a) {
    auto blendshapeMesh = _target->blendshape(bs).mesh();

    const auto col = bs - 1;

    for (auto v = 0; v < _target->numVertices(); v++) {
        const auto row = v * Eigen::Vector3d::SizeAtCompileTime;

        const auto &p = blendshapeMesh->point(blendshapeMesh->vertex_handle(_target->vertex(v)));

        for (auto i = 0; i < 3; i++) {
            a(row + i, col) = p[i];
        }
    }
}

void WeightsSolver::appendWeightFit(Index pose, VectorX &c) {
    auto poseMesh = _target->pose(pose).mesh();
    auto neutralMesh = _target->neutral();

    for (auto v = 0; v < _target->numVertices(); v++) {
        const auto row = v * Eigen::Vector3d::SizeAtCompileTime;

        const auto idx = _target->vertex(v);

        const auto &np = neutralMesh->point(neutralMesh->vertex_handle(idx));
        const auto &pp = poseMesh->point(poseMesh->vertex_handle(idx));

        const auto p = pp - np;

        for (auto i = 0; i < 3; i++) {
            c(row + i) = p[i];
        }
    }
}

void WeightsSolver::copyWeightsTo(Weights &weights, VectorX &x) {
    x.resize(weights.size() - 1);

    for (auto i = 1; i < weights.size(); i++)
        x(i - 1) = weights[i];
}

void WeightsSolver::copyWeightsTo(VectorX &x, Weights &weights) {
    double diff = 0;

    for (auto i = 0; i < x.rows(); i++) {
        const auto w = x(i, 0);
        const auto d = w - weights[i];
        diff += (d * d);

        weights[i + 1] = w;
    }

    if (_debug) {
        std::cout << "Copy Weights: " << std::endl;
        std::cout << "Diff: " << std::sqrt(diff) << std::endl;
    }
}
