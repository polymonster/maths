#!/usr/bin/env bash
c++ --std=c++11 -fprofile-arcs -ftest-coverage -fPIC -fno-inline -fno-inline-small-functions -fno-default-inline --coverage .test/test.cpp -o .test/test && ./".test/test"