#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "Compiling Leader Tests..."
g++ -std=c++17 -pthread -DUNIT_TEST -I../../ ../../lead.cpp leader_tests.cpp -o leader_tests -lOpenCL

echo "Compiling Follower Tests..."
g++ -std=c++17 -pthread -DUNIT_TEST -I../../ ../../follow.cpp follower_tests.cpp -o follower_tests -lOpenCL

echo "--------------------------------"
echo "Running Leader Tests..."
./leader_tests

echo "--------------------------------"
echo "Running Follower Tests..."
./follower_tests
