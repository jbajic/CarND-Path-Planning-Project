#!/bin/bash

mkdir -p build
pushd build
  rm -rf *
  cmake ..
  make
popd

./build/path_planning