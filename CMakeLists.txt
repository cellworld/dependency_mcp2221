cmake_minimum_required( VERSION 3.10 )

set(CMAKE_CXX_STANDARD 20)

project( mcp2221
        VERSION 2023.6.0
        DESCRIPTION "MCP2221A library for linux"
        LANGUAGES CXX)

####
#### DEPENDENCIES
####

find_package (Dependencies QUIET)

if (NOT ${Dependencies_FOUND})
    if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
        file(DOWNLOAD https://raw.githubusercontent.com/germanespinosa/dependencies/main/DependenciesConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
    endif()
    set(Dependencies_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    find_package (Dependencies REQUIRED)
endif()

####
#### END DEPENDENCIES
####

find_path(LIBUSB_INCLUDE_DIR
        NAMES libusb.h
        PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY
        NAMES usb
        PATH_SUFFIXES "lib" "lib32" "lib64")

find_package(Threads)

string(APPEND CMAKE_CXX_FLAGS "-pthread")

###
### MAIN LIBRARY SETUP
###

set(mcp2221_files
        src/hid.cpp
        src/libmcp2221.cpp
        src/mcp2221.cpp)

add_library(mcp2221 ${mcp2221_files})

add_compile_definitions(DEBUG_INFO_HID=TRUE)


set_target_properties(mcp2221
        PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED On
        CXX_EXTENSIONS Off
        VERSION ${PROJECT_VERSION})

target_compile_options(mcp2221 PRIVATE -Wall -Wextra -pedantic -ludev)

target_link_libraries(mcp2221 ${LIBUSB_LIBRARY} udev)

target_include_directories(mcp2221
        SYSTEM INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(mcp2221
        PRIVATE
        include )

###
### TESTS
###
install_git_dependency( Catch
        https://github.com/cellworld/dependency_catch
        ADD_SUBDIRECTORY)

add_catch_test( basic_test
        SOURCE_FILES catchtests/basic_tests.cpp
        INCLUDE_DIRECTORIES include
        LINK_LIBRARIES mcp2221)

add_catch_test( advanced_test
        SOURCE_FILES catchtests/advanced_test.cpp
        INCLUDE_DIRECTORIES include
        LINK_LIBRARIES mcp2221)

add_catch_test( device_test
        SOURCE_FILES catchtests/device_test.cpp
        INCLUDE_DIRECTORIES include
        LINK_LIBRARIES mcp2221)

###
### LIBRARY INSTALLATION
###

include(GNUInstallDirs)

install(TARGETS mcp2221 EXPORT Mcp2221Config
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT Mcp2221Config
        DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/mcp2221
        EXPORT_LINK_INTERFACE_LIBRARIES)

export(TARGETS mcp2221 FILE Mcp2221Config.cmake)