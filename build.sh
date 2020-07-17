#!/bin/bash

cd build
cmake ..
make
cd main
./mapbufo
cd ../..
find ./ -iname *.h -o -iname *.cpp | xargs clang-format -i -style=Google

