#/*
#-----------------------------------------------------------------------------
#Filename:    CMakeLists.txt
#-----------------------------------------------------------------------------
#
#-----------------------------------------------------------------------------
#*/
cmake_minimum_required(VERSION 2.6)

project(OgreCity)

if(WIN32)
        set(CMAKE_MODULE_PATH "$ENV{OGRE_HOME}/CMake/;${CMAKE_MODULE_PATH}")
        set(OGRE_SAMPLES_INCLUDEPATH
                $ENV{OGRE_HOME}/Samples/include
        )
endif(WIN32)

if(UNIX)
        set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
        set(OGRE_SAMPLES_INCLUDEPATH
                /usr/local/share/OGRE/samples/Common/include/
        )
endif(UNIX)

if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set(CMAKE_DEBUG_POSTFIX "_d")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")

find_package(OGRE REQUIRED)

# Add OgreTerrain.{so,dll,etc.} to our linked libraries
#   These variables are defined in FindOGRE.cmake, mine is
#   found at /usr/local/lib/OGRE/cmake/
if (OGRE_Terrain_FOUND)
# pretty sure the following include is unneeded
  # include_directories(${OGRE_Terrain_INCLUDE_DIRS})
# append OgreTerrain to the end of the OGRE_LIBRARIES variable
  set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${OGRE_Terrain_LIBRARIES})
  message(STATUS "Found OGRE_Terrain: ${OGRE_Terrain_LIBRARIES}")
else (OGRE_Terrain_FOUND)
  message(SEND_ERROR "OgreTerrain Library not found.")
endif(OGRE_Terrain_FOUND)

if(NOT "${OGRE_VERSION_NAME}" STREQUAL "Cthugha")
  message(SEND_ERROR "You need Ogre 1.7 Cthugha to build this.")
endif()

find_package(OIS REQUIRED)

if(NOT OIS_FOUND)
        message(SEND_ERROR "Failed to find OIS.")
endif()

# Find Boost
if (NOT OGRE_BUILD_PLATFORM_IPHONE)
        if (WIN32 OR APPLE)
                set(Boost_USE_STATIC_LIBS TRUE)
        else ()
                # Statically linking boost to a dynamic Ogre build doesn't work on Linux 64bit
                set(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
        endif ()
        if (MINGW)
                # this is probably a bug in CMake: the boost find module tries to look for
                # boost libraries with name libboost_*, but CMake already prefixes library
                # search names with "lib". This is the workaround.
                set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
        endif ()
        set(Boost_ADDITIONAL_VERSIONS "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
        # Components that need linking (NB does not include header-only components like bind)
        set(OGRE_BOOST_COMPONENTS thread date_time)
        find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
        if (NOT Boost_FOUND)
                # Try again with the other type of libs
                set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
                find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
        endif()
        find_package(Boost QUIET)

        # Set up referencing of Boost
        include_directories(${Boost_INCLUDE_DIR})
        add_definitions(-DBOOST_ALL_NO_LIB)
        set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
endif()

# libcity
set(LIBCIY_INCLUDE_DIR ../../libcity/include)
set(LIBCIY_LINK_DIR ../../libcity)
set(LIBCIY_LIB city)

set(HDRS
        ./ogrecity.h
        ./baseapplication.h
        ./application.h
        ./renderer.h
        ./environmentrenderer.h
        ./terrainrenderer.h
        ./streetgraphrenderer.h
        ./skyscraper.h
)

set(SRCS
        ./ogrecity.cpp
        ./baseapplication.cpp
        ./application.cpp
        ./main.cpp
        ./renderer.cpp
        ./environmentrenderer.cpp
        ./terrainrenderer.cpp
        ./streetgraphrenderer.cpp
        ./skyscraper.cpp
)

include_directories( ${OIS_INCLUDE_DIRS}
        ${OGRE_INCLUDE_DIRS}
        ${OGRE_SAMPLES_INCLUDEPATH}
        ${LIBCIY_INCLUDE_DIR}
)

link_directories(${LIBCIY_LINK_DIR})

add_executable(ogrecity WIN32 ${HDRS} ${SRCS})

set_target_properties(ogrecity PROPERTIES DEBUG_POSTFIX _d)

message(${OGRE_SAMPLES_INCLUDEPATH})

target_link_libraries(ogrecity ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${LIBCIY_LIB})

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/media)

# post-build copy for win32
if(WIN32 AND NOT MINGW)
        add_custom_command( TARGET ogrecity PRE_BUILD
                COMMAND if not exist .\\dist\\bin mkdir .\\dist\\bin )
        add_custom_command( TARGET ogrecity POST_BUILD
                COMMAND copy \"$(TargetPath)\" .\\dist\\bin )
endif(WIN32 AND NOT MINGW)

if(MINGW OR UNIX)
        set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/dist/bin)
endif(MINGW OR UNIX)

if(WIN32)

        install(TARGETS ogrecity
                RUNTIME DESTINATION bin
                CONFIGURATIONS All)

        install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/Media
                DESTINATION ./
                CONFIGURATIONS Release RelWithDebInfo Debug
        )

        install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins.cfg
                ${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
                DESTINATION bin
                CONFIGURATIONS Release RelWithDebInfo
        )

        install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins_d.cfg
                ${CMAKE_SOURCE_DIR}/dist/bin/resources_d.cfg
                DESTINATION bin
                CONFIGURATIONS Debug
        )

        # NOTE: for the 1.7.1 sdk the OIS dll is called OIS.dll instead of libOIS.dll
        # so you'll have to change that to make it work with 1.7.1
        install(FILES ${OGRE_PLUGIN_DIR_REL}/OgreMain.dll
                ${OGRE_PLUGIN_DIR_REL}/RenderSystem_Direct3D9.dll
                ${OGRE_PLUGIN_DIR_REL}/RenderSystem_GL.dll
                ${OGRE_PLUGIN_DIR_REL}/libOIS.dll
                DESTINATION bin
                CONFIGURATIONS Release RelWithDebInfo
        )

        install(FILES ${OGRE_PLUGIN_DIR_DBG}/OgreMain_d.dll
                ${OGRE_PLUGIN_DIR_DBG}/RenderSystem_Direct3D9_d.dll
                ${OGRE_PLUGIN_DIR_DBG}/RenderSystem_GL_d.dll
                ${OGRE_PLUGIN_DIR_DBG}/libOIS_d.dll
                DESTINATION bin
                CONFIGURATIONS Debug
        )

   # as of sdk 1.7.2 we need to copy the boost dll's as well
   # because they're not linked statically (it worked with 1.7.1 though)
   install(FILES ${Boost_DATE_TIME_LIBRARY_RELEASE}
      ${Boost_THREAD_LIBRARY_RELEASE}
      DESTINATION bin
      CONFIGURATIONS Release RelWithDebInfo
   )

   install(FILES ${Boost_DATE_TIME_LIBRARY_DEBUG}
      ${Boost_THREAD_LIBRARY_DEBUG}
      DESTINATION bin
      CONFIGURATIONS Debug
   )
endif(WIN32)

if(UNIX)

        install(TARGETS ogrecity
                RUNTIME DESTINATION bin
                CONFIGURATIONS All)

        install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/media
                DESTINATION ./
                CONFIGURATIONS Release RelWithDebInfo Debug
        )

        install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins.cfg
                ${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
                DESTINATION bin
                CONFIGURATIONS Release RelWithDebInfo Debug
        )

endif(UNIX)
