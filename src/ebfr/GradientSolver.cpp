//
//  GradientSolver.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "GradientSolver.h"

#include <thread>
#include <mutex>

GradientSolver::GradientSolver()
: SolverBase()
, _mSize(_Matrix::SizeAtCompileTime)
, _mCols(_Matrix::ColsAtCompileTime)
, _mRows(_Matrix::RowsAtCompileTime)
, _regK(0.1)
, _regTheta(2)
, _beta(0.5) // -> 0.1
{
    setMultithreaded(true);
}

void GradientSolver::setRegularizationConsts(const ParameterD &k, const ParameterD &theta) {
    _regK = k;
    _regTheta = theta;
}

void GradientSolver::setBlendshapeSolveConsts(const ParameterD &beta) {
    _beta = beta;
}

void GradientSolver::init() {
    calculateMStars();
    calculateWs();
}

bool GradientSolver::solve(int iter) {
    std::cout
            << std::endl
            << "Gradient Solver [" << iter << "]" << std::endl
            << "\tK: " << _regK(iter) << std::endl
            << "\tTheta: " << _regTheta(iter) << std::endl
            << "\tBeta: " << _beta(iter) << std::endl
            << std::endl;

    SolverBase::solve(iter);

    _betaIter = _beta(_iteration);

    std::mutex logMutex;

    auto solverOp =
            [this, &logMutex]
                    (int threadId, size_t faceStart, size_t faceEnd) {
                if (_debug && threadId >= 0) {
                    logMutex.lock();
                    std::cout << "Thread [" << threadId << "-GS] Start: Faces " << faceStart << " -> " << faceEnd << " (" << (faceEnd - faceStart) << ")" << std::endl;
                    logMutex.unlock();
                }

                const auto rows = (_source->numPoses() * _mSize) +
                                  (_source->numPoses() * (_source->numBlendshapes() - 1) * _mSize);
                const auto cols = _target->numBlendshapes() * _mSize;

                TripletList a;
                SparseMatrix A(rows, cols);

                MatrixX c(rows, 1);
                MatrixX x;

                a.clear();
                c.setZero();

                Index faceIndex = faceStart;
                Index numFaces = 0;

                Eigen::LLT<MatrixX> solver;

                while (true) {
                    if (faceIndex >= faceEnd)
                        break;

                    Index face = _source->face(faceIndex);

                    a.clear();
                    c.setZero();

                    appendGradientFit(face, a, c);
                    appendGradientRegularization(face, a, c);

                    A.setFromTriplets(a.begin(), a.end());
                    A.makeCompressed();

                    const auto At = A.transpose();

                    solver.compute(At * A);
                    if (!checkSolverError(solver.info()))
                        break;

                    x = solver.solve(At * c);

                    copyBlendshapeMTo(x, _targetGradients->blendshapeM, face);

                    faceIndex++;
                    numFaces++;
                }

                if (_debug && threadId >= 0) {
                    logMutex.lock();
                    std::cout << "Thread [" << threadId << "-GS] Complete - " << numFaces << " Faces" << std::endl;
                    logMutex.unlock();
                }
            };

    if (_useMultithreaded) {
        std::vector<std::thread> pool;

        const auto numThreads = std::thread::hardware_concurrency();
        const auto size = _source->numFaces();
        const auto segmentSize = (size / numThreads) + 1;
        auto segmentStart = 0;

        for (auto i = 0; i < numThreads; i++) {
            pool.push_back(
                    std::thread(
                            solverOp, i,
                            segmentStart,
                            std::min(segmentStart + segmentSize, size)
                    )
            );

            segmentStart += segmentSize;
        }

        for (auto &thread : pool) {
            thread.join();
        }
    } else {
        solverOp(-1, 0, _target->numFaces());
    }

    return true;
}

void GradientSolver::calculateMStars() {
    const auto numFaces = _source->numFaces(true);

    _mStar.resize(_source->numBlendshapes());

    const auto &s0 = _sourceGradients->blendshapeM[0];
    const auto &s0Inv = _sourceGradients->blendshapeMInv[0];

    const auto &t0 = _targetGradients->blendshapeM[0];

    for (auto bs = 1; bs < _source->numBlendshapes(); bs++) {
        auto &mStar = _mStar[bs];

        mStar.resize(numFaces);

        const auto &si = _sourceGradients->blendshapeM[bs];

        for (auto face = 0; face < numFaces; face++) {
            const auto m = (((s0[face] + si[face]) * s0Inv[face]) * t0[face]) - t0[face];
            mStar[face] = m;
        }
    }
}

void GradientSolver::calculateWs() {
    const auto numFaces = _source->numFaces(true);

    _w.resize(_source->numBlendshapes());

    const auto k = _regK(_iteration);
    const auto t = _regTheta(_iteration);

    for (auto bs = 0; bs < _source->numBlendshapes(); bs++) {
        const auto &blendshapeM = _sourceGradients->blendshapeM[bs];

        auto &blendshapeW = _w[bs];
        blendshapeW.resize(numFaces);

        for (auto face = 0; face < numFaces; face++) {
            const auto &mA = blendshapeM[face];
            const auto mAf = mA.norm();

            auto &w = blendshapeW[face];

            w = std::pow((1 + mAf) / (k + mAf), t);
        }
    }
}

void GradientSolver::appendGradientFit(Index face, TripletList &a, MatrixX &c) const {
    appendGradientFitWeights(face, a);
    appendGradientFit(face, c);
}

void GradientSolver::appendGradientFitWeights(Index face, TripletList &a) const {
    for (auto pose = 0; pose < _source->numPoses(); pose++) {
        const auto row = rowIndex(pose, false);

        for (auto bs = 0; bs < _target->numBlendshapes(); bs++) {
            const auto weight = _target->weight(pose, bs);

            if (weight == 0.0)
                continue;

            const auto col = colIndex(bs, false);

            for (auto i = 0; i < _mSize; i++) {
                a.emplace_back(row + i, col + i, weight);
            }
        }
    }
}

void GradientSolver::appendGradientFit(Index face, MatrixX &c) const {
    for (auto pose = 0; pose < _source->numPoses(); pose++) {
        appendGradientFit(face, pose, c);
    }
}

void GradientSolver::appendGradientFit(Index face, Index pose, MatrixX &c) const {
    const auto &s = _targetGradients->poseM[pose][face];
    const auto &n = _targetGradients->blendshapeM[0][face];

    const auto row = rowIndex(pose, false);

    for (auto i = 0; i < _mCols; i++) {
        for (auto j = 0; j < _mRows; j++) {
            const auto offset = (i * _mRows) + j;

            c(row + offset, 0) = s(j, i) - n(j, i);
        }
    }
}

void GradientSolver::appendGradientRegularization(Index face, TripletList &a, MatrixX &c) const {
    for (auto pose = 0; pose < _target->numPoses(); pose++) {
        appendGradientRegularization(face, pose, a, c);
    }
}

void GradientSolver::appendGradientRegularization(Index face, Index pose, TripletList &a, MatrixX &c) const {
    for (Index bs = 1; bs < _target->numBlendshapes(); bs++) {
        appendGradientRegularization(face, pose, bs, a, c);
    }
}

void GradientSolver::appendGradientRegularization(Index face, Index pose, Index bs, TripletList &a, MatrixX &c) const {
    //const Index row = rowIndex(pose, true);
    const Index col = colIndex(bs, true);

    const Index row =
            (_source->numPoses() * _mSize) +
            (((_source->numBlendshapes() - 1) * _mSize * pose) + (_mSize * (bs - 1)));

    const auto wbeta = _w[bs][face] * _betaIter;

    if (wbeta == 0.0)
        return;

    const auto &mStar = _mStar[bs][face];

    // M^B_i * wbeta
    for (auto i = 0; i < _mSize; i++) {
        a.emplace_back(row + i, col + i, wbeta);
    }

    for (auto i = 0; i < _mCols; i++) {
        for (auto j = 0; j < _mRows; j++) {
            const auto offset = (i * _mRows) + j; //index(j, i);

            c(row + offset, 0) = wbeta * mStar(j, i);
        }
    }
}

Index GradientSolver::rowIndex(Index pose, bool isReg) const {
    if (isReg)
        return (Index) (_source->numPoses() + pose) * _mSize;

    return (Index) (pose * _mSize);
}

Index GradientSolver::colIndex(Index bs, bool isReg) const {
    return (Index) (bs * _mSize);
}

Index GradientSolver::index(Index row, Index col) const {
    return (row * _mCols) + col;
}

void GradientSolver::copyBlendshapeMTo(MatrixX &x, std::vector<std::vector<Matrix3x3>> &ms, Index face) const {
    // Don't overwrite BS0/Neutral M
    auto row = _mSize;

    for (auto bs = 1; bs < _target->numBlendshapes(); bs++) {
        auto &m = ms[bs][face];

        for (auto i = 0; i < _mCols; i++) {
            for (auto j = 0; j < _mRows; j++) {
                m(j, i) = x(row, 0);
                row++;
            }
        }
    }
}
