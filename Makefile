#
# Project: OgreCity
# Date: 21.04.2011
#
# This is only a simple Makefile that calls
# CMAKE on Ogre's project.
#
# The executable is created at the executable
# path. Program can be run by writing 'make run'.
#

CMAKE_BUILD_DIRECTORY=build/
PROGRAM_EXECUTABLE_DIRECTORY=build/dist/bin/
EXECUTABLE_NAME=city3d
DOCUMENTATION_DIRECTORY=doc/

.PHONY: clean doc all build run

all: build
	cd ${CMAKE_BUILD_DIRECTORY} && make -B && make install

build:
	mkdir -p ${CMAKE_BUILD_DIRECTORY}
	cd ${CMAKE_BUILD_DIRECTORY} && cmake ../src

doc:
	mkdir -p ${DOCUMENTATION_DIRECTORY}
	doxygen Doxyfile

clean:
	rm -rf ${CMAKE_BUILD_DIRECTORY} ${DOCUMENTATION_DIRECTORY}

run:
	export LD_LIBRARY_PATH=../../../../libcity/ && cd ${PROGRAM_EXECUTABLE_DIRECTORY} && ./${EXECUTABLE_NAME}
