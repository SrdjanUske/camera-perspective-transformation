#!/bin/sh

FIND_EXEC=$(ls | grep -c perspective-transform)

if [ $FIND_EXEC -ne 0 ];
then
    echo "Rm exec..."
    rm perspective-transform
fi

echo "Build..."
g++ main.cpp IPM.cpp mouse-event.cpp -o perspective-transform -I. `pkg-config --cflags --libs opencv`
echo "Exec..."
./perspective-transform $1 $2