//
//  FS.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "FS.h"

#include <iostream>
#include <fstream>
#include <regex>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

bool Exists(const std::string &path) {
    struct stat buffer;

    return (stat(path.c_str(), &buffer) == 0);
}

bool _ListDir(const std::string &dirPath, std::vector<std::string> *files, std::vector<std::string> *dirs,
              const std::string namePattern = "", bool namesOnly = false) {
    DIR *dirp = opendir(dirPath.c_str());
    if (dirp == NULL)
        return false;

    struct dirent *dp;

    std::string basePath = dirPath.back() == '/' ? dirPath : (dirPath + "/");

    const auto hasPattern = !namePattern.empty();
    std::regex pattern(namePattern);

    while ((dp = readdir(dirp)) != NULL) {
        if (dp->d_namlen == 0)
            continue;

        std::vector<std::string> *list = nullptr;

        switch (dp->d_type) {
            case DT_DIR:
                list = dirs;
                break;
            case DT_REG:
                list = files;
                break;
        }

        if (list != nullptr) {
            if (hasPattern)
                if (!std::regex_match(dp->d_name, pattern))
                    continue;

            if (namesOnly)
                list->push_back(dp->d_name);
            else
                list->push_back(basePath + dp->d_name);
        }
    }

    closedir(dirp);

    return true;
}


bool ListDir(const std::string &dirPath, std::vector<std::string> &files, std::vector<std::string> &dirs) {
    return _ListDir(dirPath, &files, &dirs);
}

bool ListFiles(const std::string &dirPath, std::vector<std::string> &files, const std::string &ext) {
    auto success = _ListDir(dirPath, &files, nullptr);
    if (!success || ext.empty())
        return success;

    auto endIter =
            std::remove_if(
                    files.begin(), files.end(),
                    [&ext](const std::string &s) {
                        return !std::equal(ext.rbegin(), ext.rend(), s.rbegin());
                    });

    const auto numRemoved = files.end() - endIter;

    files.resize(files.size() - numRemoved);

    std::sort(files.begin(), files.end());

    return true;
}

bool ListFiles(const std::string &dirPath, std::vector<std::string> &files) {
    const auto dir = BasePath(dirPath);
    const auto filePattern = Filename(dirPath);

    auto success = _ListDir(dir, &files, nullptr, filePattern);
    if (!success)
        return success;

    static std::regex numericSortPattern(".*/(\\d*)[-\\.].*");

    std::sort(files.begin(), files.end(),
              [](const std::string &a, const std::string &b) {
                  std::smatch aMatch;
                  std::smatch bMatch;

                  if (std::regex_search(a, aMatch, numericSortPattern) && aMatch.size() > 1
                      && std::regex_search(b, bMatch, numericSortPattern) && bMatch.size() > 1) {
                      return std::atoi(aMatch.str(1).c_str()) < std::atoi(bMatch.str(1).c_str());
                  }

                  return a < b;
              });

    return true;
}

bool ListDirs(const std::string &dirPath, std::vector<std::string> &dirs) {
    return _ListDir(dirPath, nullptr, &dirs);
}

bool WriteWeights(const std::vector<double> &weights, const std::string &path) {
    std::ofstream out;
    out.open(path);

    if (!out.is_open())
        return false;

    out << weights.size() << std::endl;

    for (auto w = 0; w < weights.size(); w++)
        out << weights[w] << std::endl;

    out.close();

    return true;
}

bool WriteWeights(const std::vector<std::vector<double>> &weights, const std::string &path) {
    if (weights.empty())
        return false;

    if (weights[0].empty())
        return false;

    std::ofstream out;
    out.open(path);

    if (!out.is_open())
        return false;

    out << weights.size() << "," << weights[0].size() << std::endl;

    for (const auto &ws : weights) {
        for (auto w : ws) {
            out << w << std::endl;
        }
    }

    out.close();

    return true;
}

//bool WriteFltFile(const ImageMap& img, const std::string& path, bool sparse, bool includeId)
//{
//    std::ofstream file;
//    file.open(path);
//    
//    if (!file.is_open())
//        return false;
//    
//    if (sparse)
//        file << "S";
//    else
//        file << "D";
//    
//    if (includeId)
//        file << "I";
//    
//    file << std::endl;
//    
//    const auto maxDepth = img.maxDepth();
//    
//    file << img.width() << "," << img.height() << "," << maxDepth << ",3" << std::endl;
//    
//    for (auto j = 0; j < img.height(); j++)
//    {
//        for (auto i = 0; i < img.width(); i++)
//        {
//            const auto& list = img.get(i, j);
//            
//            for (auto n = 0; n < list.size(); n++)
//            {
//                const auto& entry = list[n];
//                
//                if (sparse)
//                {
//                    if (entry.p.isZero())
//                        continue;
//                    
//                    file << i << "," << j << "," << n;
//                    
//                    for (auto c = 0; c < 3; c++)
//                    {
//                        file << "," << entry.p[c];
//                    }
//                    
//                    if (includeId)
//                    {
//                        file << "," << entry.id;
//                    }
//                    
//                    file << std::endl;
//                }
//                else
//                {
//                    for (auto c = 0; c < 3; c++)
//                    {
//                        file << entry.p[c] << std::endl;
//                    }
//                    
//                    if (includeId)
//                    {
//                        file << entry.id << std::endl;
//                    }
//                }
//            }
//            
//            if (!sparse)
//            {
//                for (auto n = list.size(); n < maxDepth; n++)
//                {
//                    for (auto c = 0; c < 3; c++)
//                    {
//                        file << "0" << std::endl;
//                    }
//                    
//                    if (includeId)
//                    {
//                        file << "-1" << std::endl;
//                    }
//                }
//            }
//        }
//    }
//    
//    file.close();
//    
//    return true;
//}
//
//bool ReadFltFile(ImageMap& img, const std::string& path)
//{
//    std::ifstream file;
//    file.open(path);
//    
//    if (!file.is_open())
//    {
//        std::cerr << "Failed to open FLT file - [" << path << "]" << std::endl;
//        return false;
//    }
//    
//    char comma;
//    
//    char formatType, idType;
//    file >> formatType >> idType;
//    
//    int width, height, depth, components;
//    file >> width >> comma >> height >> comma >> depth >> comma >> components;
//    
//    img.resize(width, height);
//    
//    bool includesId = idType == 'I';
//    
//    if (formatType == 'D')
//    {
//        for (auto j = 0; j < height; j++)
//        {
//            for (auto i = 0; i < width; i++)
//            {
//                for (auto d = 0; d < depth; d++)
//                {
//                    ImageMap::Entry entry;
//                    
//                    for (auto c = 0; c < 3; c++)
//                    {
//                        file >> entry.p[c];
//                    }
//                    
//                    if (includesId)
//                        file >> entry.id;
//                    
//                    img.push(i, j, entry.p, entry.id);
//                }
//            }
//        }
//    }
//    else
//    {
//        while (!file.eof())
//        {
//            int x, y, n;
//            file >> x >> comma >> y >> comma >> n;
//            
//            ImageMap::Entry entry;
//            
//            for (auto c = 0; c < 3; c++)
//                file >> comma >> entry.p[c];
//            
//            if (includesId)
//                file >> entry.id;
//            
//            img.push(x, y, entry.p, entry.id);
//        }
//    }
//    
//    return true;
//}
