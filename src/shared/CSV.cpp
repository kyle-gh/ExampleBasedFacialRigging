//
//  CSV.cpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#include "CSV.h"

#include <algorithm>
#include <sstream>

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

CSV::CSV()
        : _delim(',') {

}

bool CSV::open(const std::string &path) {
    _file.open(path);

    if (!_file.is_open())
        return false;

    return readHeader();
}

void CSV::close() {
    if (_file.is_open())
        _file.close();
}

bool CSV::next() {
    if (!_file.is_open())
        return false;

    if (_file.eof())
        return false;

    std::getline(_file, _line);

    std::istringstream ss;
    ss.str(_line);

    for (auto i = 0; !ss.eof() && i < _numColumns; i++) {
        std::getline(ss, _values[i], _delim);
        trim(_values[i]);
    }

    return true;
}

const std::string &CSV::value(int index) const {
    return _values[index];
}

const std::vector<std::string> &CSV::values() const {
    return _values;
}

bool CSV::values(const std::vector<std::string> &names, std::vector<std::string> &values) {
    if (_columns.empty())
        return false;

    values.resize(names.size());

    for (auto i = 0; i < names.size(); i++)
        values[i] = _values[_columns[names[i]]];

    return true;
}

bool CSV::values(int start, int end, std::vector<std::string> &values) {
    const auto len = end - start;

    values.resize(len);

    for (auto i = 0; i < len; i++)
        values[i] = _values[start + i];

    return true;
}

bool CSV::readHeader() {
    if (_file.eof())
        return false;

    std::getline(_file, _line);

    std::istringstream ss;
    ss.str(_line);

    std::string name;

    _numColumns = 0;

    for (auto i = 0; !ss.eof(); i++) {
        std::getline(ss, name, _delim);

        _columns[name] = i;
        _numColumns++;
    }

    _values.resize(_numColumns);

    return true;
}

bool PoseCSV::Write(const std::string &path, const std::vector<std::vector<double>> &weights) {
    std::ofstream csv;
    csv.open(path);

    // Header
    csv << "Pose,";
    for (auto i = 0; i < weights.size(); i++) {
        csv << i << ",";
    }
    csv << std::endl;

    for (auto i = 0; i < weights.size(); i++) {
        if (i != 0) {
            csv << std::endl;
        }

        csv << i << ",";

        for (auto w : weights[i]) {
            csv << w << ",";
        }
    }

    csv.close();

    return true;
}

PoseCSV::PoseCSV()
        : CSV() {

}

bool PoseCSV::open(const std::string &path) {
    return CSV::open(path);
}

bool PoseCSV::next() {
    return CSV::next();
}

bool PoseCSV::values(std::string &name, std::vector<double> &weights) {
    auto &v = CSV::values();

    name = v[0];

    weights.resize(v.size() - 1);

    for (auto i = 1; i < v.size(); i++) {
        weights[i - 1] = v[i].empty() ? 0.0 : std::stod(v[i]);
    }

    return true;
}

