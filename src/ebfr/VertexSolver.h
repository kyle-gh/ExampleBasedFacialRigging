//
//  VertexSolver.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef VertexSolver_hpp
#define VertexSolver_hpp

#include "SolverBase.h"

class VertexSolver : public SolverBase {
public:
    VertexSolver();

    virtual void init();

    virtual bool solve(int iter);

    bool solve(int iter, int bs);

private:
    typedef Matrix3x3 _Matrix;
    typedef Vector3 _Vector;

    typedef Eigen::Matrix<double, 3, 4> MatrixE;
    typedef Eigen::Matrix<double, _Matrix::SizeAtCompileTime, 1> MatrixQ;
    typedef Eigen::SimplicialLDLT<SparseMatrix> Solver;

    const size_t _mSize;
    const size_t _mCols;
    const size_t _mRows;

    const size_t _vSize;

    const bool _usePhantom;
    const double _fixedWeight;
    const int _maxFixed;
    const bool _randomFixed;

    std::vector<std::vector<int>> _fixedVertices;

    class SolverData {
    public:
        SolverData()
                : initialized(false)
                , at()
                , c()
                , x()
                , solver()
            {}

        SolverData(const SolverData &other)
                : SolverData()
        {}

        bool initialized;

        SparseMatrix at;

        MatrixX c;

        MatrixX x;

        Solver solver;
    };

    std::vector<SolverData> _solvers;

    StepCallback _callback;

    bool transfer(Index bs);

    void constructA(int bs, SparseMatrix &a);

    void constructA(Index face, const MatrixE &e, TripletList &m);

    void constructFixedA(Index bs, TripletList &m);

    void constructE(Index face, MatrixE &e) const;

    void constructC(Index bs, MatrixX &c);

    void constructC(Index face, const Matrix3x3 &q, MatrixX &c);

    void constructFixedC(Index bs, MatrixX &c);

    void copyTo(Index bs, MatrixX &x) const;

    Index vertexIndex(Index idx) const;

    void vertexIndices(Index face, Index vertices[]) const;

    bool checkSolverError(const Solver &solver) const;
};

#endif /* VertexSolver_hpp */
