#!/bin/bash

colcon build \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    # --symlink-install

# -DCMAKE_BUILD_TYPE=RelWithDebInfo \
