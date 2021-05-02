//
//  FS.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef FS_hpp
#define FS_hpp

#include <vector>
#include <string>

//#include "ImageMap.h"

bool Exists(const std::string &path);

bool ListAll(const std::string &dirPath, std::vector<std::string> &files, std::vector<std::string> &dirs);

bool ListFiles(const std::string &dirPath, std::vector<std::string> &files);

bool ListDirs(const std::string &dirPath, std::vector<std::string> &dirs);

bool WriteWeights(const std::vector<double> &weights, const std::string &path);

bool WriteWeights(const std::vector<std::vector<double>> &weights, const std::string &path);

//bool WriteFltFile(const ImageMap& img, const std::string& path, bool sparse, bool includeId = false);
//
//bool ReadFltFile(const ImageMap& img, const std::string& path);

// The base case: we just have a single number.
inline const std::string &JoinPath(const std::string &s) {
    return s;
}

// The recursive case: we take a number, alongside
// some other numbers, and produce their sum.
template<typename... Rest>
inline std::string JoinPath(const std::string &t, Rest... rest) {
    if (t.empty())
        return JoinPath(rest...);
    else {
        if (t[t.size() - 1] == '/')
            return t + JoinPath(rest...);
        else
            return t + "/" + JoinPath(rest...);
    }
}

inline std::string ReplaceExt(const std::string &path, const std::string &ext) {
    const auto index = path.find_last_of('.');

    if (index == std::string::npos)
        return path + "." + ext;

    return path.substr(0, index + 1) + ext;
}

inline std::string RemoveExt(const std::string &path) {
    const auto index = path.find_last_of('.');

    if (index == std::string::npos)
        return path;

    return path.substr(0, index);
}

inline std::string BasePath(const std::string &path) {
    auto index = path.find_last_of('/');

    if (index == std::string::npos)
        return "/";

    if (index == path.size() - 1) {
        index = path.find_last_of('/', index - 1);

        if (index == std::string::npos)
            return "/";
    }

    return path.substr(0, index + 1);
}

inline std::string Filename(const std::string &path, bool withExtension = true) {
    const auto index = path.find_last_of('/');

    if (index == std::string::npos)
        return path;

    if (!withExtension) {
        const auto extIndex = path.find_last_of('.');

        return path.substr(index + 1, extIndex - index - 1);
    }

    return path.substr(index + 1, path.size() - index - 1);
}

#endif /* FS_hpp */
