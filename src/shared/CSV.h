//
//  CSV.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef CSV_hpp
#define CSV_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

class CSV {
public:
    CSV();

    bool open(const std::string &path);

    void close();

    bool next();

    const std::vector<std::string> &values() const;

    const std::string &value(int index) const;

    bool values(const std::vector<std::string> &names, std::vector<std::string> &values);

    bool values(int start, int end, std::vector<std::string> &values);

private:
    char _delim;

    std::ifstream _file;

    int _numColumns;

    std::map<std::string, int> _columns;

    std::string _line;

    std::vector<std::string> _values;

    bool readHeader();
};

class PoseCSV : protected CSV {
public:
    static bool Write(const std::string &path, const std::vector<std::vector<double>> &weights);

    PoseCSV();

    bool open(const std::string &path);

    bool next();

    bool values(std::string &name, std::vector<double> &weights);
};

#endif /* CSV_hpp */
