#!/bin/bash
# cd $HOME
# wget https://github.com/Kitware/CMake/releases/download/v3.29.7/cmake-3.29.7-linux-x86_64.sh

# chmod +x cmake-3.29.7-linux-x86_64.sh
# sudo ./cmake-3.29.7-linux-x86_64.sh --prefix=/usr/local --skip-license
#rm -f ./cmake-3.29.7-linux-x86_64.sh

SOPHUS_VERSION=1.22.10
SOPHUS_DIR=$HOME/Sophus

cd $HOME
if [ ! -d "$SOPHUS_DIR/.git" ]; then
  git clone https://github.com/strasdat/Sophus.git "$SOPHUS_DIR"
fi

cd "$SOPHUS_DIR"
git fetch --tags
git checkout "$SOPHUS_VERSION"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j6
sudo cmake --install build
# cd $HOME
# rm -rf Sophus
