#!/bin/bash

# clean build
if [ -d "build" ]; then
	echo "clean build "
	rm -r build/
else
	echo "no build"
fi

# clean log
if [ -d "log" ]; then
	echo "clean log "
	rm -r log/
else
	echo "no log"
fi

# clean install
if [ -d "install" ]; then
	echo "clean install "
	rm -r install/
else
	echo "no install"
fi

# clean build-arm
if [ -d "build-arm" ]; then
	echo "clean build-arm "
	rm -r build-arm/
else
	echo "no build-arm"
fi

# clean bin-arm
if [ -d "bin-arm" ]; then
	echo "clean bin-arm "
	rm -r bin-arm/
else
	echo "no bin-arm"
fi
