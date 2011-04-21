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
PROGRAM_EXECUTABLE_PATH=build/dist/bin/ogrecity
DOCUMENTATION_DIRECTORY=doc/

.PHONY: clean doc all run

all:
	cd ${CMAKE_BUILD_DIRECTORY} && cmake ../src
	cd ${CMAKE_BUILD_DIRECTORY} && make -B && make install

doc:
	mkdir -p ${DOCUMENTATION_DIRECTORY}
	doxygen Doxyfile

clean:
	cd ${CMAKE_BUILD_DIRECTORY} && rm -rf
