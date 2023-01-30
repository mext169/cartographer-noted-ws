#!/bin/sh
if [ -d build ]; then
	cd build
	make -j6
	cd ..
else
	echo "[build]不存在 先运行clean_and_make.sh"
fi

