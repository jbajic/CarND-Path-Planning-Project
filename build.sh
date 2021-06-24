#!/bin/bash

set -euo pipefail

arg=${1:-}

mkdir -p build
pushd build
  rm -rf *
  if [[ ${arg} == "DEBUG" ]]; then
    echo Debugging
    cmake -DCMAKE_BUILD_TYPE=DEBUG ..
    make
    gdb --args ./path_planning
  else
    echo Running
    cmake ..
    make
    ./path_planning
  fi
popd