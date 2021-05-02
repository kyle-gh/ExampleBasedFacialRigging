//
//  Rig.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "Rig.h"

#include "../shared/FS.h"
#include "../shared/CSV.h"
#include "../shared/Mesh.h"

#include <fstream>
#include <random>

void Rig::load(const std::string& dirPath, const std::string& posePath, const std::string& weightsPath, const std::string& vertexMaskPath, bool isTarget)
{
    std::cout
            << "Reading " << (isTarget ? "Target" : "Source") << " Rig" << std::endl
            << "\t" << dirPath << std::endl;

    if (isTarget) {
        loadNeutral(dirPath);
    } else {
        loadBlendshapes(dirPath);
    }

    PoseCSV csv;
    if (!csv.open(weightsPath)) {
        std::cerr << "Failed to open Pose CSV " << weightsPath << std::endl;
        return;
    }

    std::cout
            << "\tPoses" << std::endl
            << "\t\t" << posePath << std::endl;

    std::vector<std::string> posePaths;
    std::vector<std::vector<double>> poseWeights;

    std::string poseName;
    std::vector<double> poseWeight;

    while (csv.next()) {
        csv.values(poseName, poseWeight);

        if (!std::any_of(poseWeight.begin(), poseWeight.end(), [](double w) { return w > 0.0; }))
            continue;

        // Neutral/BS0
        poseWeight.insert(poseWeight.begin(), 1);

        posePaths.push_back(JoinPath(posePath, poseName + ".obj"));
        poseWeights.push_back(poseWeight);

        std::cout << "\t\t" << poseName << " - ";
        for (auto w : poseWeight)
            std::cout << w << " ";
        std::cout << std::endl;
    }

    loadPoses(posePaths, poseWeights);

    if (!vertexMaskPath.empty()) {
        loadVertexMask(vertexMaskPath);

        std::cout << "Modified Verticies: " << vertices().size() << " / " << numVertices(true) << std::endl;
        std::cout << "Modified Faces: " << faces().size() << " / " << numFaces(true) << std::endl;
    }

    return;
}

void Rig::loadBlendshapes(const std::string &dirPath) {
    std::vector<std::string> blendshapePaths;
    ListFiles(JoinPath(dirPath, "(\\d*).obj"), blendshapePaths);
    blendshapePaths.insert(blendshapePaths.begin(), JoinPath(dirPath, "neutral.obj"));

    loadBlendshapes(blendshapePaths);
}

void Rig::loadBlendshapes(const std::vector<std::string> &paths) {
    _blendshapes.resize(paths.size());

    for (auto i = 0; i < paths.size(); i++) {
        _blendshapes[i].setMesh(ReadMesh(paths[i]), i > 0);
    }
}

void Rig::loadNeutral(const std::string &path) {
    if (_blendshapes.empty()) {
        _blendshapes.resize(1);
    }

    _blendshapes[0].setMesh(ReadMesh(path), false);
}

void Rig::generateEmptyBlendshapes(size_t num) {
    _blendshapes.resize(num);

    for (auto i = 0; i < num; i++) {
        if (_blendshapes[i].mesh() != nullptr) {
            continue;
        }

        auto mesh = MakeMesh(_poses[0].mesh());

        SetVertices(mesh, Mesh::Point(0, 0, 0));

        _blendshapes[i].setMesh(mesh, false);
    }
}

void Rig::randomizeWeights() {
    std::mt19937 g(0);

    std::normal_distribution<> a{0, 0.5};
    std::normal_distribution<> b{0, 0.1};

    for (auto i = 0; i < numPoses(); i++) {
        auto &ws = weights(i);

        for (auto &w : ws) {
            w = std::clamp(w * (1.0 - a(g)) + std::abs(b(g)), 0.0, 1.0);
        }
    }
}

MeshPtr Rig::generatePose(int pose) const {
    return generatePose(weights(pose));
}

MeshPtr Rig::generatePose(const std::vector<double> &weights) const {
    auto target = MakeMesh(neutral());

    for (auto bs = 1; bs < numBlendshapes(); bs++) {
        AddVertices(target, blendshape(bs).mesh(), weights[bs], target);
    }

    return target;
}

void Rig::loadPoses(const std::vector<std::string> &paths, const std::vector<Weights> &weights) {
    _poses.resize(paths.size());

    for (auto i = 0; i < paths.size(); i++) {
        auto mesh = ReadMesh(paths[i]);

        _poses[i].setMesh(mesh);
        _poses[i].setWeights(weights[i]);
    }
}

void Rig::loadVertexMask(const std::string &path) {
    std::ifstream file;
    file.open(path);

    int vid;
    while (!file.eof()) {
        file >> vid;
        _vertices.emplace_back(vid);
    }

    file.close();

    auto mesh = _poses[0].mesh();

    for (auto v : _vertices) {
        const auto vh = mesh->vertex_handle(v);

        for (auto iter = mesh->cvf_begin(vh), end = mesh->cvf_end(vh); iter != end; iter++) {
            _faces.emplace_back(iter->idx());
        }
    }

    std::sort(_faces.begin(), _faces.end());
    auto lastF = std::unique(_faces.begin(), _faces.end());
    if (lastF != _faces.end()) {
        _faces.erase(lastF, _faces.end());
    }

    _vertices.clear();
    _vertices.reserve(_faces.size() * 3);

    for (auto f : _faces) {
        const auto fh = mesh->face_handle(f);

        for (auto iter = mesh->cfv_begin(fh), end = mesh->cfv_end(fh); iter != end; iter++) {
            _vertices.emplace_back(iter->idx());
        }
    }

    std::sort(_vertices.begin(), _vertices.end());

    auto lastV = std::unique(_vertices.begin(), _vertices.end());
    if (lastV != _vertices.end()) {
        _vertices.erase(lastV, _vertices.end());
    }
}

void Rig::findModified() {
    const auto numV = numVertices();

//    for (auto bs = 1; bs < _blendshapes.size(); bs++)
//    {
//        const auto& isFixed = _blendshapes[bs].isFixed();
//
//        for (auto i = 0; i < numV; i++)
//        {
//            if (!isFixed[i]) {
//                if (i == 0) {
//                    std::cout << _blendshapes[bs].mesh()->point(_blendshapes[bs].mesh()->vertex_handle(i)) << std::endl;
//                    std::cout <<std::endl;
//                }
//                _vertices.emplace_back(i);
//            }
//        }
//    }

    const auto neutral = _blendshapes[0].mesh()->points();
    for (auto pose : _poses) {
        const auto points = pose.mesh()->points();

        for (auto i = 0; i < numV; i++) {
            const auto diff = points[i] - neutral[i];
            if (!isNearZero(diff, 0.5)) {
                _vertices.emplace_back(i);
            }
        }
    }

    std::sort(_vertices.begin(), _vertices.end());
    auto lastV = std::unique(_vertices.begin(), _vertices.end());
    if (lastV != _vertices.end()) {
        _vertices.erase(lastV, _vertices.end());
    }


    auto mesh = _poses[0].mesh();

    for (auto v : _vertices) {
        const auto vh = mesh->vertex_handle(v);

        for (auto iter = mesh->cvf_begin(vh), end = mesh->cvf_end(vh); iter != end; iter++) {
            _faces.emplace_back(iter->idx());
        }
    }

    std::sort(_faces.begin(), _faces.end());
    auto lastF = std::unique(_faces.begin(), _faces.end());
    if (lastF != _faces.end()) {
        _faces.erase(lastF, _faces.end());
    }
}

size_t Rig::faceIndex(size_t index) const {
    if (_faces.empty()) {
        return index;
    }

    auto iter = std::lower_bound(_faces.begin(), _faces.end(), index);
    if (iter == _faces.end()) {
        return 0;
    }

    return iter - _faces.begin();
}

size_t Rig::vertexIndex(size_t index) const {
    if (_vertices.empty()) {
        return index;
    }

    auto iter = std::lower_bound(_vertices.begin(), _vertices.end(), index);
    if (iter == _vertices.end()) {
        return 0;
    }

    return iter - _vertices.begin();
}
