#!/bin/bash

cd ..
make refdump TEST=private/$1.c
cp refdump.reg private/$1.reg
rm refdump.reg
make verify TEST=private/$1.c