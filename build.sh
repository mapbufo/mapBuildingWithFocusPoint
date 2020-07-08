#!/bin/bash

cd build
cmake ..
make
find ./ -iname *.h -o -iname *.cpp | xargs clang-format -i -style=Google
cd main
./mapbufo

