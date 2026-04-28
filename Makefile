SHELL = /bin/bash

# acceptable build_types: Release/Debug/Profile
build_type=Release

.SILENT:

all: build-only install

install: build/CMakeLists.txt.copy
	$(MAKE) --no-print-directory -C build install

build-only: build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

clean:
	rm -rf build bin lib install

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) -DCMAKE_INSTALL_PREFIX=../install ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build
