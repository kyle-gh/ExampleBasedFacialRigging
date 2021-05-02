//
//  SolverUtil.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef SolverUtil_hpp
#define SolverUtil_hpp

#include "Mesh.h"
#include "Matrix.h"

void CalculateSurface(const Mesh &ref, const Mesh::FaceHandle &refFace, Matrix3x3 &s);

void CalculateInvSurface(const Mesh &ref, const Mesh::FaceHandle &refFace, Matrix3x3 &inv);

void ConstructTriangleNormMatrix(const Mesh &mesh, const Mesh::FaceHandle &face, Matrix3x3 &v);

void CalculateFrames(MeshPtr mesh, std::vector<Matrix3x3> &gradients);

void CalculateInvFrames(MeshPtr mesh, std::vector<Matrix3x3> &gradients);

void GenerateEmptyFrames(MeshPtr, std::vector<Matrix3x3> &gradients);

#endif /* SolverUtil_hpp */
