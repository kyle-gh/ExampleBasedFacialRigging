//
//  Mesh.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "Mesh.h"

MeshPtr ReadMesh(const std::string &path, bool exitOnFail) {
    auto mesh = MakeMesh();

    Mesh &meshRef = *mesh;

    meshRef.request_face_normals();
    meshRef.request_vertex_normals();
    //meshRef.request_vertex_texcoord();

    OpenMesh::IO::Options opts(OpenMesh::IO::Options::VertexNormal | OpenMesh::IO::Options::FaceNormal |
                               OpenMesh::IO::Options::VertexTexCoord);
    if (!OpenMesh::IO::read_mesh(meshRef, path, opts)) {
        std::cerr << "Failed to read mesh at [" << path << "]" << std::endl;

        if (exitOnFail)
            exit(1);

        return nullptr;
    }

    meshRef.update_face_normals();
    meshRef.update_vertex_normals();

    return mesh;
}

bool WriteMesh(const std::string &path, MeshPtr mesh) {
    if (!OpenMesh::IO::write_mesh(*mesh, path)) {
        std::cerr << "Failed to write mesh to [" << path << "]" << std::endl;
        return false;
    }

    return true;
}
