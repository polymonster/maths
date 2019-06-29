#!/usr/bin/env bash
c++ --std=c++11 -fprofile-arcs -ftest-coverage -fPIC .test/test.cpp -o .test/test && ./".test/test"