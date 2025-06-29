#!/bin/sh

files="main.c lc3vm.c"
flags=-g

gcc $flags $files -o lc3vm
