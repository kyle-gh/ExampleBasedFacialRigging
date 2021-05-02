//
//  GradientSolver.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef GradientSolver_hpp
#define GradientSolver_hpp

#include "SolverBase.h"

class GradientSolver : public SolverBase {
public:
    GradientSolver();

    void setRegularizationConsts(const ParameterD &k, const ParameterD &theta);

    void setBlendshapeSolveConsts(const ParameterD &beta);

    virtual void init();

    virtual bool solve(int iter);

private:
    typedef Matrix3x3 _Matrix;

    const Index _mSize;
    const Index _mCols;
    const Index _mRows;

    ParameterD _regK;
    ParameterD _regTheta;
    ParameterD _beta;

    double _betaIter;

    std::vector<std::vector<double>> _w;

    std::vector<std::vector<Matrix3x3>> _mStar;

    void calculateMStars();

    void calculateWs();

    void appendGradientFit(Index face, TripletList &a, MatrixX &c) const;

    void appendGradientFitWeights(Index face, TripletList &a) const;

    void appendGradientFit(Index face, MatrixX &c) const;

    void appendGradientFit(Index face, Index pose, MatrixX &c) const;

    void appendGradientFit(Index face, Index pose, Index bs, MatrixX &c) const;

    void appendGradientRegularization(Index face, TripletList &a, MatrixX &c) const;

    void appendGradientRegularization(Index face, Index pose, TripletList &a, MatrixX &c) const;

    void appendGradientRegularization(Index face, Index pose, Index bs, TripletList &a, MatrixX &c) const;

    Index rowIndex(Index pose, bool isReg) const;

    Index colIndex(Index bs, bool isReg) const;

    Index index(Index row, Index col) const;

    void copyBlendshapeMTo(MatrixX &x, std::vector<std::vector<Matrix3x3>> &ms, Index face) const;
};

#endif /* GradientSolver_hpp */
