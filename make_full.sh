#!/bin/sh
if [ -d build ]; then
	rm -rf build
	rm -rf devel
fi
mkdir build	
cd build
cmake .. -DCATKIN_DEVEL_PREFIX=../devel
make -j4
