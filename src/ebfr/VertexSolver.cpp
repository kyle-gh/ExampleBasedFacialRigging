//
//  VertexSolver.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "VertexSolver.h"

#include "../shared/Timing.h"

#include <random>

VertexSolver::VertexSolver()
: SolverBase()
, _mSize(_Matrix::SizeAtCompileTime)
, _mCols(_Matrix::ColsAtCompileTime)
, _mRows(_Matrix::RowsAtCompileTime)
, _vSize(_Vector::SizeAtCompileTime)
, _usePhantom(false)
, _fixedWeight(0.5)
, _maxFixed(100)
, _randomFixed(true)
{

}

void VertexSolver::init() {
    _fixedVertices.resize(_source->numBlendshapes());

    for (auto bs = 1; bs < _source->numBlendshapes(); bs++) {
        const auto blendshape = _source->blendshape(bs);

        auto &fixedVertices = _fixedVertices[bs];

        fixedVertices.resize(blendshape.numFixed());
        std::copy(blendshape.fixed().begin(), blendshape.fixed().end(), fixedVertices.begin());

        if (_maxFixed != -1 && _maxFixed < fixedVertices.size()) {
            if (_randomFixed) {
                std::mt19937 g(0);

                std::shuffle(fixedVertices.begin(), fixedVertices.end(), g);

                fixedVertices.resize(_maxFixed);

                std::sort(fixedVertices.begin(), fixedVertices.end());
            } else {
                fixedVertices.resize(_maxFixed);
            }
        }
    }

    _solvers.resize(_target->numBlendshapes());
    for (auto &solver : _solvers) {
        solver.initialized = false;
    }
}

bool VertexSolver::solve(int iter) {
    return solve(iter, -1);
}

bool VertexSolver::solve(int iter, int bs) {
    std::cout
            << std::endl
            << "Vertex Solver [" << iter << "]"
            << std::endl;

    SolverBase::solve(iter);

    if (bs != -1) {
        transfer(bs);
    } else {
        for (bs = 1; bs < _target->numBlendshapes(); bs++) {
            transfer(bs);
        }
    }

    if (_callback != nullptr)
        _callback(iter, _target, _debugPath);

    return true;
}

bool VertexSolver::transfer(Index bs) {
    if (_debug) {
        std::cout << "\tBlendshape [" << bs << "]" << std::endl;
    }

    auto &solver = _solvers[bs];

    if (!solver.initialized) {
        SparseMatrix a;

        constructA(bs, a);

        TIMER_START(Compute);

        solver.at = a.transpose();

        solver.solver.compute(solver.at * a);

        TIMER_END(Compute);

        if (!checkSolverError(solver.solver)) {
            std::cerr << "Vertex Solver failed to init" << std::endl;
            return false;
        }

        solver.initialized = true;
    }

    constructC(bs, solver.c);

    solver.x = solver.solver.solve(solver.at * solver.c);

    if (!checkSolverError(solver.solver))
        return false;

    copyTo(bs, solver.x);

    return true;
}

void VertexSolver::constructA(int bs, SparseMatrix &a) {
    auto rows = _target->numFaces(true) * _mSize;

    rows += _fixedVertices[bs].size() * _vSize;

    auto cols = _target->numVertices(true) * _vSize;

    if (_usePhantom) {
        cols += _target->numFaces(true) * _vSize;
    }

    if (_debug) {
        std::cout
                << "Constructing A" << std::endl
                << "\tSize: " << rows << " x " << cols << std::endl;
    }

    TripletList m;
    m.reserve(rows * 10);

    MatrixE e;

    for (auto face = 0; face < _target->numFaces(true); face++) {
        constructE(face, e);

        constructA(face, e, m);
    }

    constructFixedA(bs, m);

    a.resize(rows, cols);
    a.setZero();

    a.setFromTriplets(m.begin(), m.end());

    a.makeCompressed();
}

void VertexSolver::constructE(const Index face, MatrixE &e) const {
    const auto &mInv = _targetGradients->blendshapeMInv[0][face];

    e << -mInv(0, 0) - mInv(1, 0) - (_usePhantom ? mInv(2, 0) : 0),
            mInv(0, 0),
            mInv(1, 0),
            mInv(2, 0),

            -mInv(0, 1) - mInv(1, 1) - (_usePhantom ? mInv(2, 1) : 0),
            mInv(0, 1),
            mInv(1, 1),
            mInv(2, 1),

            -mInv(0, 2) - mInv(1, 2) - (_usePhantom ? mInv(2, 2) : 0),
            mInv(0, 2),
            mInv(1, 2),
            mInv(2, 2);
}

void VertexSolver::constructA(const Index face, const MatrixE &e, TripletList &m) {
    const auto numVerts = _usePhantom ? 4 : 3;

    Index vertexIdx[4];
    vertexIndices(face, vertexIdx);

    auto row = face * _mSize;

    for (int coord = 0; coord < 3; coord++) {
        for (int eqn = 0; eqn < 3; eqn++, row++) {
            for (int vert = 0; vert < numVerts; vert++) {
                const int vertIdx = (int) vertexIdx[vert];

                m.emplace_back(row, vertIdx + coord, e(eqn, vert));
            }
        }
    }
}

void VertexSolver::constructFixedA(Index bs, TripletList &m) {
    const auto &fixed = _fixedVertices[bs];

    auto row = (int) (_target->numFaces() * _mSize);

    for (auto i : fixed) {
        auto idx = vertexIndex(i);

        for (auto j = 0; j < 3; j++) {
            m.emplace_back(row, idx, _fixedWeight);

            row++;
            idx++;
        }
    }
}

void VertexSolver::constructC(Index bs, MatrixX &c) {
    auto rows = _target->numFaces(true) * 9;

    rows += _fixedVertices[bs].size() * _vSize;

    if (_debug) {
        std::cout
                << "Constructing C" << std::endl
                << "\tSize: " << rows << " x " << 1 << std::endl;
    }

    c.resize(rows, 1);
    c.setZero();

    const auto &neutralM = _targetGradients->blendshapeM[0];
    const auto &neutralMInv = _targetGradients->blendshapeMInv[0];
    const auto &blendshapeM = _targetGradients->blendshapeM[bs];

    const auto I = Matrix3x3::Identity();

    for (auto face = 0; face < _target->numFaces(true); face++) {
        const auto &m = blendshapeM[face];

        if (m.isZero()) {
            constructC(face, I, c);
        } else {
            constructC(face, (neutralM[face] + m) * neutralMInv[face], c);
        }
    }

    constructFixedC(bs, c);
}

void VertexSolver::constructC(const Index face, const Matrix3x3 &q, MatrixX &c) {
    auto row = face * MatrixQ::RowsAtCompileTime;

    for (auto i = 0; i < 3; i++) {
        for (auto j = 0; j < 3; j++) {
            c(row, 0) = q(i, j);
            row++;
        }
    }
}

void VertexSolver::constructFixedC(Index bs, MatrixX &c) {
    const auto &fixed = _fixedVertices[bs];

    auto mesh = _target->neutral();

    auto row = (int) (_target->numFaces() * _mSize);

    for (auto i : fixed) {
        const auto vert = mesh->vertex_handle(i);
        const auto &p = mesh->point(vert);

        for (auto j = 0; j < 3; j++) {
            c(row, 0) = _fixedWeight * p[j];

            row++;
        }
    }
}

void VertexSolver::copyTo(Index bs, MatrixX &x) const {
    auto target = _target->blendshape(bs).mesh();

    for (auto v = 0; v < _target->numVertices(true); v++) {
        const auto vh = target->vertex_handle(v);
        const auto idx = vertexIndex(v);

        target->point(vh) = OpenMesh::Vec3d(x(idx, 0), x(idx + 1, 0), x(idx + 2, 0));;
    }

    if (_debug) {
        auto neutral = _target->neutral();
        const auto &fixedVertices = _fixedVertices[bs];

        auto printVert = [&target, &neutral](auto vert, bool flagged = false) {
            const auto p = target->point(target->vertex_handle(vert));
            const auto np = neutral->point(neutral->vertex_handle(vert));

            std::cout << "\tVert " << vert << ": " << p << " :: " << p - np << (flagged ? " *" : "") << std::endl;
        };

        std::cout << "Vertices: " << std::endl;
        printVert(target->n_vertices() / 3);
        printVert(target->n_vertices() / 2);
        printVert(2 * (target->n_vertices() / 3));

        if (!fixedVertices.empty()) {
            std::cout << "Fixed: " << std::endl;
            printVert(fixedVertices[fixedVertices.size() / 3]);
            printVert(fixedVertices[fixedVertices.size() / 2]);
            printVert(2 * (fixedVertices[fixedVertices.size() / 3]));
        }
    }
}

Index VertexSolver::vertexIndex(Index idx) const {
    return (Index) (idx * _vSize);
}

void VertexSolver::vertexIndices(const Index face, Index vertices[]) const {
    const auto mesh = _target->blendshape(0).mesh();
    const auto fh = mesh->face_handle(face);

    size_t i = 0;
    for (auto vertIter = mesh->cfv_begin(fh), vertEnd = mesh->cfv_end(fh);
         i < 3 && vertIter != vertEnd; vertIter++, i++) {
        vertices[i] = vertexIndex(vertIter->idx());
    }

    vertices[3] = (Index) ((_target->numVertices(true) + fh.idx()) * _vSize);
}

bool VertexSolver::checkSolverError(const Solver &solver) const {
    return SolverBase::checkSolverError(solver.info());
}
