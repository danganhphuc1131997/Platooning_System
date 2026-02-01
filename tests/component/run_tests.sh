#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "Compiling Component Tests..."
# We need to link output object files or include cpp files directly.
# Including cpp files is easier for simple test scripts to avoid makefile complexity for now.
# We include lead.cpp and follow.cpp but we might need to be careful about main() function in them.
# Wait, lead.cpp and follow.cpp likely HAVE a main function if they are standalone executables.
# Let's check if they have main(). If so, we cannot include them directly without -DUNIT_TEST guarding main.

# I previously used -DUNIT_TEST for defect tests, assuming the main() in lead.cpp/follow.cpp is guarded.
# Let's verify that assumption.

grep -q "ifdef UNIT_TEST" ../../lead.cpp || echo "Warning: UNIT_TEST guard might be missing in lead.cpp"
grep -q "ifdef UNIT_TEST" ../../follow.cpp || echo "Warning: UNIT_TEST guard might be missing in follow.cpp"

g++ -std=c++17 -pthread -DUNIT_TEST -I../../ ../../lead.cpp ../../follow.cpp component_tests.cpp -o component_tests -lOpenCL

echo "--------------------------------"
echo "Running Component Tests..."
./component_tests
