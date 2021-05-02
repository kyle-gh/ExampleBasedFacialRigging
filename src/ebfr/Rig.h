//
//  Rig.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef Rig_hpp
#define Rig_hpp

#include "../shared/Mesh.h"

#include <stdio.h>
#include <memory>

//class Weights : public std::vector<double>
//{};
typedef std::vector<double> Weights;

class Pose {
public:
    Pose()
            : _mesh(nullptr) {
    }

    Pose(MeshPtr pose)
            : _mesh(pose) {
    }

    Pose(MeshPtr mesh, const Weights &w)
            : _mesh(mesh), _weights(w) {
    }

    MeshPtr mesh() const { return _mesh; }

    const Weights &weights() const { return _weights; }

    Weights &weights() { return _weights; }

    void setMesh(MeshPtr pose) {
        _mesh = pose;
    }

    void setWeights(const Weights &w) {
        _weights = w;
    }

private:
    MeshPtr _mesh;

    Weights _weights;
};

class Blendshape {
public:
    Blendshape()
            : _mesh(nullptr) {
    }

    Blendshape(MeshPtr blendshape)
            : _mesh(nullptr) {
        setMesh(blendshape);
    }

    MeshPtr mesh() const { return _mesh; }

    void setMesh(MeshPtr blendshape, bool checkFixed = true) {
        _mesh = blendshape;

        if (checkFixed) {
            for (auto vertIter = _mesh->vertices_begin(), vertEnd = _mesh->vertices_end();
                 vertIter != vertEnd; vertIter++) {
                const auto vert = *vertIter;

                if (isNearZero(_mesh->point(vert), 0.5)) {
                    _fixed.emplace_back(vert.idx());
                }
            }
        }
    }

    size_t numFixed() const {
        return _fixed.size();
    }

    bool isFixed(int v) const {
        return std::binary_search(_fixed.begin(), _fixed.end(), v);
    }

    const std::vector<int> &fixed() const {
        return _fixed;
    }

private:
    MeshPtr _mesh;

    std::vector<int> _fixed;
};

class Rig {
public:
    void load(const std::string &dirPath, const std::string &posePath, const std::string &weightsPath,
              const std::string &vertexMaskPath, bool isTarget);

    void loadBlendshapes(const std::string &dirPath);

    void loadBlendshapes(const std::vector<std::string> &paths);

    void loadNeutral(const std::string &path);

    void loadPoses(const std::vector<std::string> &paths, const std::vector<Weights> &weights);

    void loadVertexMask(const std::string &path);

    void findModified();

    void generateEmptyBlendshapes(size_t num);

    void randomizeWeights();

    MeshPtr generatePose(int pose) const;

    MeshPtr generatePose(const std::vector<double> &weights) const;

    MeshPtr neutral() const { return _blendshapes[0].mesh(); }

    size_t numVertices(bool all = false) const {
        return _vertices.empty() || all ? neutral()->n_vertices() : _vertices.size();
    }

    size_t numFaces(bool all = false) const { return _faces.empty() || all ? neutral()->n_faces() : _faces.size(); }

    const std::vector<int> &vertices() const { return _vertices; }

    const std::vector<int> &faces() const { return _faces; }

    size_t face(size_t index) const { return _faces.empty() ? index : _faces[index]; }

    size_t faceIndex(size_t index) const;

    size_t vertex(size_t index) const { return _vertices.empty() ? index : _vertices[index]; }

    size_t vertexIndex(size_t index) const;

    std::vector<Blendshape> &blendshapes() { return _blendshapes; }

    const std::vector<Blendshape> &blendshapes() const { return _blendshapes; }

    Blendshape &blendshape(int bs) { return _blendshapes[bs]; }

    const Blendshape &blendshape(int bs) const { return _blendshapes[bs]; }

    size_t numBlendshapes() const { return _blendshapes.size(); }

    std::vector<Pose> &poses() { return _poses; }

    const std::vector<Pose> &poses() const { return _poses; }

    Pose &pose(int pose) { return _poses[pose]; }

    const Pose &pose(int pose) const { return _poses[pose]; }

    size_t numPoses() const { return _poses.size(); }

    std::vector<Weights> weights() const {
        std::vector<Weights> weights;
        for (const auto &pose : _poses) { weights.push_back(pose.weights()); }
        return weights;
    }

    Weights &weights(int pose) { return _poses[pose].weights(); }

    const Weights &weights(int pose) const { return _poses[pose].weights(); }

    double weight(int pose, int bs) const { return weights(pose)[bs]; }

    size_t numActiveWeights(int pose) const {
        return std::count_if(weights(pose).begin(), weights(pose).end(), [](double v) { return v > 0.0; });
    }

private:
    std::vector<Blendshape> _blendshapes;

    std::vector<Pose> _poses;

    std::vector<int> _vertices;
    std::vector<int> _faces;
};

typedef std::shared_ptr<Rig> RigPtr;

inline RigPtr MakeRig() {
    return std::make_shared<Rig>();
}

#endif /* Rig_hpp */
