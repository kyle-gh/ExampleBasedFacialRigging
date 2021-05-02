//
//  Mesh.h
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef Mesh_h
#define Mesh_h

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <memory>

#include <iostream>

// OpenMesh Mesh Traits
// Use Double-based structures
struct DoubleTraits : OpenMesh::DefaultTraits {
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
};

typedef OpenMesh::TriMesh_ArrayKernelT<DoubleTraits> Mesh;

typedef std::shared_ptr<Mesh> MeshPtr;

inline MeshPtr MakeMesh(MeshPtr copy = nullptr) {
    auto mesh = std::make_shared<Mesh>();
    if (copy != nullptr)
        *mesh = *copy;

    return mesh;
}

inline bool isNan(const OpenMesh::Vec3d &v) {
    return std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2]);
}

inline bool isZero(const OpenMesh::Vec3d &v) {
    return v[0] == 0.0 && v[1] == 0.0 && v[2] == 0.0;
}

inline bool isNearZero(const OpenMesh::Vec3d &v, double eps = 0.00001) {
    return std::abs(v[0]) < eps && std::abs(v[1]) < eps && std::abs(v[2]) < eps;
}

inline void FaceVertices(const Mesh &mesh, const Mesh::FaceHandle &face, Mesh::VertexHandle vertices[]) {
    size_t i = 0;
    for (auto vertIter = mesh.cfv_begin(face), vertEnd = mesh.cfv_end(face);
         i < 3 && vertIter != vertEnd; vertIter++, i++) {
        vertices[i] = (*vertIter);
    }
}

inline void Adjacent(const Mesh &mesh, const Mesh::FaceHandle &face, Mesh::FaceHandle adjacent[]) {
    size_t i = 0;
    for (auto adj_iter = mesh.cff_begin(face), adj_end = mesh.cff_end(face);
         i < 3 && adj_iter != adj_end; i++, adj_iter++) {
        adjacent[i] = *adj_iter;
    }
}

inline void Bounds(const Mesh &mesh, OpenMesh::Vec3d &min, OpenMesh::Vec3d &max) {
    min = mesh.point(mesh.vertex_handle(0));
    max = min;

    for (auto vert_iter = mesh.vertices_begin(), vert_end = mesh.vertices_end(); vert_iter != vert_end; vert_iter++) {
        const auto &p = mesh.point(*vert_iter);

        min.minimize(p);
        max.maximize(p);
    }
}

inline void SetVertices(MeshPtr target, const Mesh::Point &p) {
    for (auto vert_iter = target->vertices_begin(), vert_end = target->vertices_end();
         vert_iter != vert_end; vert_iter++) {
        const auto vert = *vert_iter;

        target->set_point(vert, p);
    }
}

inline void CopyVertices(MeshPtr target, MeshPtr source) {
    const auto *src = source->points();

    for (auto vert_iter = target->vertices_begin(), vert_end = target->vertices_end();
         vert_iter != vert_end; vert_iter++) {
        const auto vert = *vert_iter;

        target->set_point(vert, src[vert.idx()]);
    }
}

inline void CopyVertices(MeshPtr target, MeshPtr source, double weight) {
    const auto *src = source->points();

    for (auto vert_iter = target->vertices_begin(), vert_end = target->vertices_end();
         vert_iter != vert_end; vert_iter++) {
        const auto vert = *vert_iter;

        target->set_point(vert, weight * src[vert.idx()]);
    }
}

inline void AddVertices(MeshPtr base, MeshPtr modifier, double weight, MeshPtr dest = nullptr) {
    if (dest == nullptr)
        dest = base;

    const auto numVertices = std::min(base->n_vertices(), modifier->n_vertices());
    for (auto i = 0; i < numVertices; i++) {
        const auto tVert = base->vertex_handle(i);
        const auto sVert = modifier->vertex_handle(i);
        const auto dVert = dest->vertex_handle(i);

        auto sp = modifier->point(sVert);
        sp *= weight;

        auto tp = base->point(tVert);
        tp += sp;

        if (isNearZero(tp)) {
            tp = Mesh::Point(0, 0, 0);
        }

        dest->point(dVert) = tp;
    }
}

MeshPtr ReadMesh(const std::string &path, bool exitOnFail = true);

bool WriteMesh(const std::string &path, MeshPtr mesh);

#endif /* Mesh_h */
