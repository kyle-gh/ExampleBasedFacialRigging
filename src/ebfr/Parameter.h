//
//  Parameter.hpp
//  ExampleBasedFacialRigging
//
//  Created by Kyle on 5/1/21.
//  Copyright © 2021 Kyle. All rights reserved.
//

#ifndef Parameter_hpp
#define Parameter_hpp

#include <stdio.h>
#include <algorithm>

template<typename T>
class Parameter {
public:
    enum Type {
        Step,
        Continuous,
    };

    struct Entry {
        int time;
        T value;
    };

    Parameter(T v) {
        setConstant(v);
    }

    Parameter(Type type, const std::vector<Entry>& entries)
    : _type(type)
    , _schedule(entries)
    , _isModified(true)
    {
    }

    void setConstant(T v) {
        _schedule.push_back(Entry{-1, v});
        _isModified = true;
    }

    void setType(Type type) {
        _type = type;
    }

    void addEntry(int t, T v) {
        _schedule.push_back(Entry{t, v});
        _isModified = true;
    }

    T operator()(int t) {
        if (_schedule.empty())
            return (T) 0;

        if (_schedule[0].time == -1)
            return _schedule[0].value;

        if (_isModified) {
            std::sort(_schedule.begin(), _schedule.end(),
                      [](const Entry &l, const Entry &r) { return l.time < r.time; });
            _isModified = false;
        }

        auto index = _schedule.size();

        for (auto i = 0; i < _schedule.size(); i++) {
            if (t < _schedule[i].time) {
                index = i;
            }
        }

        if (index == 0) {
            return _schedule[0].value;
        } else if (index == _schedule.size()) {
            return _schedule[index - 1].value;
        } else {
            if (_type == Step) {
                return _schedule[index - 1].value;
            } else if (_type == Continuous) {
                const auto &from = _schedule[index - 1];
                const auto &to = _schedule[index];

                auto i = (double) (t - from.time) / (double) (to.time - from.time);

                return ((to.value - from.value) * i) + from.value;
            }
        }

        return (T) 0;
    }

private:
    bool _isModified;

    Type _type;

    std::vector<Entry> _schedule;
};

typedef Parameter<double> ParameterD;

#endif /* Parameter_hpp */
