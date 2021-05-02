//
//  SolverUtil.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "SolverUtil.h"

#include "Util.h"

void CalculateSurface(const Mesh &ref, const Mesh::FaceHandle &refFace, Matrix3x3 &s) {
    ConstructTriangleNormMatrix(ref, refFace, s);
}

void CalculateInvSurface(const Mesh &ref, const Mesh::FaceHandle &refFace, Matrix3x3 &inv) {
    Matrix3x3 Vr;
    CalculateSurface(ref, refFace, Vr);

    inv = Vr.inverse();
}

void ConstructTriangleNormMatrix(const Mesh &mesh, const Mesh::FaceHandle &face, Matrix3x3 &v) {
    Mesh::VertexHandle vertices[3];

    FaceVertices(mesh, face, vertices);

    const auto v0 = mesh.point(vertices[0]);
    const auto v1 = mesh.point(vertices[1]);
    const auto v2 = mesh.point(vertices[2]);

    const auto e0 = toEigen(v1 - v0);
    const auto e1 = toEigen(v2 - v0);
    if (e0.isZero() && e1.isZero()) {
        v.setZero();
    } else {
        const auto n = e0.cross(e1).normalized();

        v.col(0) = e0;
        v.col(1) = e1;
        v.col(2) = n;
    }
}

void CalculateFrames(MeshPtr mesh, std::vector<Matrix3x3> &gradients) {
    gradients.resize(mesh->n_faces());

    for (auto faceIter = mesh->faces_begin(), faceEnd = mesh->faces_end(); faceIter != faceEnd; faceIter++) {
        const auto face = *faceIter;

        CalculateSurface(*mesh, face, gradients[face.idx()]);
    }
}

void CalculateInvFrames(MeshPtr mesh, std::vector<Matrix3x3> &gradients) {
    gradients.resize(mesh->n_faces());

    for (auto faceIter = mesh->faces_begin(), faceEnd = mesh->faces_end(); faceIter != faceEnd; faceIter++) {
        const auto face = *faceIter;

        CalculateInvSurface(*mesh, face, gradients[face.idx()]);
    }
}

void GenerateEmptyFrames(MeshPtr mesh, std::vector<Matrix3x3> &gradients) {
    gradients.resize(mesh->n_faces());

    for (auto &m : gradients) {
        m.setZero();
    }
}