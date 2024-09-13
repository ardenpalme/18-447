#!/bin/bash

echo "make verify TEST=benchmarks/mixed.c"
make verify TEST=benchmarks/mixed.c > mixed_output.txt

echo "benchmarks/mmmRV32IM.c"
make verify TEST=benchmarks/mmmRV32IM.c > mmmRV32IM_output.txt

echo "benchmarksO3/mmmRV32IM.c"
make verify TEST=benchmarksO3/mmmRV32IM.c > mmmRV32IMO3_output.txt


