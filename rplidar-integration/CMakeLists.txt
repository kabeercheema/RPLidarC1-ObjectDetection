cmake_minimum_required(VERSION 2.8.12)

################################################################################
##     This file is part of RTMaps                                            ##
##     Copyright (c) Intempora S.A. All rights reserved.                      ##
################################################################################
# USAGE
#
# Related article on Intempora's knowledge base:
#     * https://support.intempora.com/hc/en-us/articles/360021738293
#
# 1. Windows and Linux: CLion / QtCreator / KDevelop / Visual Studio Code
#     * These IDEs natively support CMake-based projects on Windows and Linux
#     * Simply open this folder in the IDE
#     * Use the IDE's interface to add a CMake cache variable named RTMAPS_SDKDIR
#     * that contains the path to the RTMaps installation directory
#     * On Windows, configure the IDE to use Visual C++ (MSVC) as the compiler
#
# 2. Windows: Visual Studio
#     1. Configuring the project: Open a command prompt in your project's directory then:
#         mkdir build
#         cd build
#         cmake -G"<generator name>" [ -D"RTMAPS_SDKDIR=<RTMaps Install Dir>" ] ..
#         * <generator name>
#             A supported Visual Studio-based generator (e.g. "Visual Studio 14 2015 Win64")
#             Use "cmake --help" to get the list of valid generators
#         * <RTMaps Install Dir>
#             Path to the RTMaps installation directory (e.g. "C:/Program Files/Intempora/RTMaps 4")
#             If this argument is not specified, the value of the RTMAPS_SDKDIR "environment variable" will be used
#
#     2. Compiling the project:
#         1. Either using the Visual Studio IDE:
#             Simply open the generated .sln file in the Visual Studio IDE
#         1. Or using the command line:
#             cmake --build . [ --config (Release|Debug|RelWithDebInfo) ] [ --target <Target Name> ]
#             * <Target Name>
#                 Name of the package's target in the CMakeLists.txt file
#
# 3. Linux: Command Line
#     1. Configuring the project: Open a shell in your project's directory then:
#         mkdir build
#         cd build
#         cmake [ -D"RTMAPS_SDKDIR=<RTMaps Install Dir>" ] [ -DCMAKE_BUILD_TYPE=(Release|Debug|RelWithDebInfo) ] ..
#         * <RTMaps Install Dir>
#             Path to the RTMaps installation directory (e.g. "/opt/rtmaps")
#             If this argument is not specified, the value of the RTMAPS_SDKDIR "environment variable" will be used
#
#     2. Compiling the project using the command line
#         cmake --build . [ --config (Release|Debug|RelWithDebInfo) ] [ --target <Target Name> ]
#         * <Target Name>
#             Name of the package's target in the CMakeLists.txt file
#
# 4. Linux: Cross Compiling
#     Cross compilation is similar to compiling for Linux (i.e. "3. Linux: Command Line")
#     The main difference is that you need to specify the paths to 2 additional CMake files;
#     * CMAKE_TOOLCHAIN_FILE : Board-specific, standard CMake toolchain file https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html
#     * RTMAPS_TOOLCHAIN_FILE: A file that contains RTMaps-specific compilation options for the board
#
#     1. Configuring the project: Open a shell in your project's directory then:
#         mkdir build
#         cd build
#         cmake -D"CMAKE_TOOLCHAIN_FILE=<CMake Toolchain File>" \
#               -D"RTMAPS_TOOLCHAIN_FILE=<RTMaps Target Options File>" \
#               [ -D"RTMAPS_SDKDIR=<RTMaps Install Dir>" ] [ -DCMAKE_BUILD_TYPE=(Release|Debug|RelWithDebInfo) ] ..
#         * <CMake Toolchain File>
#             Path to the board's CMake toolchain file
#         * <RTMaps Target Options File>
#             Path the file that contains RTMaps-specific compilation options for the board
#         * <RTMaps Install Dir>
#             Path to the RTMaps installation directory (e.g. "/opt/rtmaps")
#             If this argument is not specified, the value of the RTMAPS_SDKDIR "environment variable" will be used
#
#     2. Compiling the project using the command line
#         cmake --build . [ --config (Release|Debug|RelWithDebInfo) ] [ --target <Target Name> ]
#         * <Target Name>
#             Name of the package's target in the CMakeLists.txt file
#

set(PCK TestLidar)
project("${PCK}.u")

## IMPORTANT: Set the same C++ standard for the whole project ######################################

set(CMAKE_CXX_STANDARD          11)   # Proper C++ >= 11 is required
set(CMAKE_CXX_STANDARD_REQUIRED ON)   # Enforce the C++ standard requirement
set(CMAKE_CXX_EXTENSIONS        OFF)  # Non-portable compiler-specific extensions are NOT required


## IMPORTANT: Locate the RTMaps SDK ################################################################

if (NOT DEFINED RTMAPS_SDKDIR)
    if (DEFINED ENV{RTMAPS_SDKDIR})
        set(RTMAPS_SDKDIR "$ENV{RTMAPS_SDKDIR}" CACHE PATH "")  # Use the default RTMaps installation
    else()
        message(FATAL_ERROR "RTMAPS_SDKDIR not defined. Either pass -D\"RTMAPS_SDKDIR=...\" to CMake or define an RTMAPS_SDKDIR environment variable")
    endif()
endif()

include("${RTMAPS_SDKDIR}/templates.u/rtmaps.cmake")

## IMPORTANT: Locate the input reader library ######################################################

if (NOT TARGET rtmaps_input_reader)
    set(rtmaps_input_reader_DIR "${RTMAPS_SDKDIR}/packages/rtmaps_input_reader/lib/cmake/rtmaps_input_reader" CACHE PATH "")
    find_package(rtmaps_input_reader CONFIG REQUIRED NO_DEFAULT_PATH)
endif()

# OPTIONAL: Search for third-party libraries that are "natively" supported by CMake
# * https://cmake.org/Wiki/CMake:How_To_Find_Libraries
# * https://cmake.org/cmake/help/v3.0/command/find_package.html
#
#     # Example 1: Boost http://www.boost.org/
#     set(BOOST_ROOT "path/to/boost")  # OPTIONAL: force CMake to search for Boost in that directory
#     find_package(Boost REQUIRED)
#
#     # Example 2: OpenCV
#     find_package(OpenCV 3.3.0 REQUIRED PATHS "path/to/OpenCVConfig.cmake/directory")  # OPTIONAL: require version 3.3.0 and specify the DIRECTORY of OpenCVConfig.cmake

# OPTIONAL: Import third-party libraries that do not provide CMake configuration files
# * https://cmake.org/Wiki/CMake/Tutorials/Exporting_and_Importing_Targets
# * https://cmake.org/cmake/help/latest/command/add_library.html
#
#     # Example 1: A STATIC library called "foo"
#     add_library(foo STATIC IMPORTED)
#     set(foo_INCLUDE_DIR "path/to/foo/include")
#     if (MSVC)
#         set_property(TARGET foo PROPERTY IMPORTED_LOCATION "path/to/foo.lib")
#     else()
#         set_property(TARGET foo PROPERTY IMPORTED_LOCATION "path/to/libfoo.a")
#     endif()
#
#     # Example 2: A SHARED library called "bar"
#     add_library(bar SHARED IMPORTED)
#     set(bar_INCLUDE_DIR "path/to/bar/include")
#     if (MSVC)
#         set_property(TARGET bar PROPERTY IMPORTED_LOCATION "path/to/bar.dll")
#         set_property(TARGET bar PROPERTY IMPORTED_IMPLIB   "path/to/bar.lib")
#     else()
#         set_property(TARGET bar PROPERTY IMPORTED_LOCATION "path/to/libbar.so")
#     endif()

# REQUIRED: Create the pck target
# Note that the following folders will be automatically added to the list of include directories
# * local_interfaces
# * protected_interfaces
# * public_interfaces
add_rtmaps_package(${PCK} PCKINFO "${PCK}.pckinfo"
    # add sources, headers and/or directories

    "local_interfaces"                    # NB: if you add and/or remove files to this directory, you must re-run the CMake generation command
    "src"                                 # NB: if you add and/or remove files to this directory, you must re-run the CMake generation command

    # other examples
    # "src_generated/some_file.hpp"
    # "src_generated/another_file.cpp"
    # "../external_directory"             # NB: if you add and/or remove files to this directory, you must re-run the CMake generation command
)

if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")
endif()


# Link the package with the input reader library
target_link_libraries(${PCK} rtmaps_input_reader)
#target_include_directories(${PCK} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../rplidar_sdk/sdk/include)
target_include_directories(${PCK} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../rplidar_sdk/sdk/src)
target_include_directories(${PCK} PRIVATE ${RTMAPS_SDKDIR}/sdk/vc.net2022_x64/include)


# Adjust paths to match your actual SDK build output
set(RPLIDAR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../rplidar_sdk/sdk/include")
set(RPLIDAR_LIB_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../rplidar_sdk/output/x64/Release/rplidar_driver.lib")

# Add header search path
target_include_directories(${PCK} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../rplidar_sdk/sdk/include)

# Import the static library into CMake
add_library(rplidar_sdk STATIC IMPORTED)
set_property(TARGET rplidar_sdk PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../rplidar_sdk/output/x64/Release/rplidar_driver.lib)

# Link it into your RTMaps package
target_link_libraries(${PCK}
    rplidar_sdk
)

# On Windows, for TCP/UDP support, also link with WinSock
if (WIN32)
    target_link_libraries(${PCK} ws2_32)
endif()

# === RPLIDAR SDK integration done ===

# OPTIONAL: (CMake >= 3.8.0) Automatic generation of source groups
# (shows well-organized source tree in IDEs such as Visual Studio)
# get_target_property(PCK_SOURCES ${PCK} SOURCES)
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" FILES ${PCK_SOURCES})  # https://cmake.org/cmake/help/v3.8/command/source_group.html

# OPTIONAL: Use third-party libraries
#
#     # Example: Using multiple libraries
#     target_include_directories(${PCK} PRIVATE
#         "${Boost_INCLUDE_DIR}"
#         ${OpenCV_INCLUDE_DIRS}
#         "${foo_INCLUDE_DIR}"
#         "${bar_INCLUDE_DIR}"
#     )
#     target_link_libraries(${PCK}
#         ${Boost_LIBRARIES}
#         ${OpenCV_LIBS}
#         foo
#         bar
#     )

# OPTIONAL: Use RTMaps-provided libraries
# target_link_libraries(${PCK} rtmaps_candb_facade)


## RECOMMENDED: Good compiler options ##############################################################
#
#if (MSVC)
#    # MSVC Toolset >= 140 (i.e. 2015) is required
#    target_compile_options(${PCK} PRIVATE /MP /W4 /WX)
#else()
#    # https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
#    target_compile_options(${PCK} PRIVATE -fno-strict-aliasing -fdiagnostics-show-option -Wall -Wextra -pedantic -Wstrict-aliasing -Wshadow -Wtrigraphs -Werror)
#endif()
