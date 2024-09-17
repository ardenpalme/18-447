#!/bin/bash

cd ..
make verify TEST=benchmarks/mixed.c > mixed_output.txt &
make verify TEST=benchmarks/mmmRV32IM.c > mmmRV32IM_output.txt &
make verify TEST=benchmarksO3/mmmRV32IM.c > mmmRV32IMO3_output.txt &
# make view-timing > timing.rpt
# make view-area > area.rpt
# make view-power > power.rpt