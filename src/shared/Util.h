//
//  Util.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef Util_hpp
#define Util_hpp

#include "Mesh.h"
#include "Matrix.h"

inline Vector3 toEigen(const OpenMesh::Vec3d &v) {
    return Vector3(v[0], v[1], v[2]);
}

#endif /* Util_hpp */
