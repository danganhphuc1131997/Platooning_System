#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "Compiling Defect Tests..."
g++ -std=c++17 -pthread -DUNIT_TEST -I../../ ../../lead.cpp ../../follow.cpp defect_tests.cpp -o defect_tests -lOpenCL

echo "--------------------------------"
echo "Running Defect Tests..."
./defect_tests
