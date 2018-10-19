#/usr/bin/env bash

g++ -std=c++17 -I/usr/include/eigen3 test.cpp -o main
./main | tee distance.log

gnuplot -e "
    plot \"distance.log\" using 2:5 w lp, \"distance.log\" using 2:11 w lp, \"distance.log\" using 2:17 w lp;
    pause mouse, key;
"

rm main
