//
//  Matrix.h
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef Matrix_h
#define Matrix_h

#include <Eigen/SparseCholesky>
#include <Eigen/Dense>
#include <Eigen/Sparse>

typedef Eigen::Triplet<double> Triplet;
typedef std::vector<Triplet> TripletList;

typedef Eigen::SparseMatrix<double> SparseMatrix;

typedef Eigen::Matrix<double, 3, 4> Matrix3x4;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3;
typedef Eigen::Matrix<double, 3, 2> Matrix3x2;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2;
typedef Eigen::Matrix<double, 9, 4> Matrix9x4;
typedef Eigen::Matrix<double, 6, 4> Matrix6x4;
typedef Eigen::Matrix<double, 9, 1> Matrix9x1;
typedef Eigen::Matrix<double, 6, 1> Matrix6x1;

typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorX;


#ifndef EPS
#define EPS 0.0001
#endif


template<int n_rows, int n_cols>
void QRDecompose(const Eigen::Matrix<double, n_rows, n_cols> &m, Eigen::Matrix<double, n_rows, n_cols> &q,
                 Eigen::Matrix<double, n_cols, n_cols> &r) {
    Eigen::Matrix<double, n_rows, 1> v;
    Eigen::Matrix<double, n_rows, 1> qi;

    r.setZero();

    for (int j = 0; j < n_cols; j++) {
        v = m.col(j);

        for (int i = 0; i < j; i++) {
            qi = q.col(i);

            r(i, j) = qi.dot(v);
            v = v - r(i, j) * qi;
        }

        auto l = v.norm();

        if (l < EPS) {
            r(j, j) = 1;
            q.col(j).setZero();
        } else {
            r(j, j) = l;
            q.col(j) = v / l;
        }
    }
}

template<int n_rows, int n_cols>
Eigen::Matrix<double, n_cols, n_rows> Invert(const Eigen::Matrix<double, n_rows, n_cols> &m) {
    Eigen::Matrix<double, n_rows, n_cols> q;
    Eigen::Matrix<double, n_cols, n_cols> r;

    QRDecompose<n_rows, n_cols>(m, q, r);

    return r.inverse() * q.transpose();
}

inline std::string Error(Eigen::ComputationInfo info) {
    switch (info) {
        case Eigen::Success:
            return "Success";

            /** The provided data did not satisfy the prerequisites. */
        case Eigen::NumericalIssue:
            return "Numerical Issue";

            /** Iterative procedure did not converge. */
        case Eigen::NoConvergence:
            return "No Convergence";

            /** The inputs are invalid, or the algorithm has been improperly called.
             * When assertions are enabled, such errors trigger an assert. */
        case Eigen::InvalidInput:
            return "Invalid Input";
    }

    return "Unknown";
}

#endif /* Matrix_h */
