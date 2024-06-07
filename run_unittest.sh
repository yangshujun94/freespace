#!/bin/bash

cd unit_test
rm -rf build
mkdir build
cd build
cmake ..
make -j $(($(nproc)-2))

cd ..
./build/unittest_fs --gtest_output=xml:result.xml
./build/unittest_fs --gtest_output=json:result.json

COVERAGE_FILE=coverage.info
REPORT_FOLDER=coverage_report
lcov --rc lcov_branch_coverage=1 -c -d build -o ${COVERAGE_FILE}_tmp
lcov --rc lcov_branch_coverage=1  -e ${COVERAGE_FILE}_tmp "*src*" -o ${COVERAGE_FILE}
genhtml --rc genhtml_branch_coverage=1 ${COVERAGE_FILE} -o ${REPORT_FOLDER}
rm -rf ${COVERAGE_FILE}_tmp
rm -rf ${COVERAGE_FILE}

