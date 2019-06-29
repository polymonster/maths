#!/usr/bin/env bash
c++ --std=c++11 -ftest-coverage .test/test.cpp -o .test/test && ./".test/test"