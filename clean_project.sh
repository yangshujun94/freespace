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
