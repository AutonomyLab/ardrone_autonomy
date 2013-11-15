#!/bin/sh
rm -rf ./lib
rm -rf ARDroneLib/Soft/Build/targets_versions
mkdir ./lib
make -f sdk.makefile
