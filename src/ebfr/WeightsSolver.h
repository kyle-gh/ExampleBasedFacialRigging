//
//  WeightsSolver.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef WeightsSolver_hpp
#define WeightsSolver_hpp

#include "SolverBase.h"

#include <unsupported/Eigen/LevenbergMarquardt>

class WeightsSolver : public SolverBase {
public:
    struct WeightsFunctor : Eigen::DenseFunctor<double> {
        MatrixX *a;
        VectorX *c;
        //VectorX vDiff;

        VectorX estimateW;
        //VectorX wDiff;

        double lambda;

        VectorX x;

        int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

        int inputs() const;

        int values() const;
    };

    struct WeightsFunctorNumericalDiff : Eigen::NumericalDiff<WeightsFunctor> {
    };

    WeightsSolver();

    void setLambda(const ParameterD &lambda);

    virtual void init();

    virtual bool solve(int iter);

private:
    const int _maxIterations;
    const double _minWeight;
    const double _maxWeight;

    ParameterD _lambda;

    std::vector<VectorX> _estimateWs;

    MatrixX _a;
    std::vector<VectorX> _Cs;

    StepCallback _callback;

    void initSolverData(WeightsFunctor &data);

    void appendWeightFit(Index Pose, MatrixX &a, VectorX &c);

    void appendWeightFit(Index pose, MatrixX &a);

    void appendWeightFit(Index pose, Index bs, MatrixX &a);

    void appendWeightFit(Index pose, VectorX &c);

    void copyWeightsTo(Weights &weights, VectorX &x);

    void copyWeightsTo(VectorX &x, Weights &weights);
};

#endif /* WeightsSolver_hpp */
