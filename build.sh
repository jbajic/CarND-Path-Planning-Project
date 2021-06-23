#!/bin/bash

arg=${1:-}

mkdir -p build
pushd build
  rm -rf *
  if [[ ${arg} -eq DEBUG ]]; then
      cmake -DCMAKE_BUILD_TYPE=DEBUG..
      make
    ./build/path_planning
  else
    cmake ..
    make
  ./build/path_planning
  fi
popd