#!/usr/bin/env bash
c++ --std=c++20 -Wno-braced-scalar-init .test/test.cpp -o .test/test && ./".test/test"
c++ --std=c++17 -Wno-braced-scalar-init .test/test.cpp -o .test/test && ./".test/test"
c++ --std=c++14 -Wno-braced-scalar-init .test/test.cpp -o .test/test && ./".test/test"
c++ --std=c++11 -Wno-braced-scalar-init -fprofile-arcs -ftest-coverage -fPIC -fno-inline -fno-inline-small-functions -fno-default-inline --coverage .test/test.cpp -o .test/test && ./".test/test"