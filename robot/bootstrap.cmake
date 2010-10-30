##
## bootstrap.cmake
##
## Author(s):
##  - Cedric GESTES <gestes@aldebaran-robotics.com>
##
## Copyright (C) 2009 Aldebaran Robotics
##
##

###############################################
# Warning:                                    #
# This file should stay verbatin              #
# This file is part of the T001CHAIN project  #
###############################################

set(BOOTSTRAP_VERSION 1)
if (NOT CMAKE_TOOLCHAIN_FILE)
  message(STATUS
    "\n"
    "===========================================================\n"
    "= No toolchain file has been specified                    =\n"
    "= Please use cmake-gui (>= 2.6.4)                         =\n"
    "= or call cmake like that:                                =\n"
    "= cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake ..      =\n"
    "= with a valid toolchain.cmake                            =\n"
    "= Please refer to the T001chain doc                       =\n"
    "===========================================================\n"
    )
  message(FATAL_ERROR "")
endif (NOT CMAKE_TOOLCHAIN_FILE)

#ok I must admin 1337 15 n07 7h47 gr347 f0r c0d1ng
#for the export of TOOLCHAIN_DIR to T001CHAIN_DIR
if (NOT TOOLCHAIN_DIR STREQUAL "")
  set(T001CHAIN_DIR ${TOOLCHAIN_DIR} CACHE PATH "" FORCE)
endif (NOT TOOLCHAIN_DIR STREQUAL "")

if (NOT T001CHAIN_DIR OR NOT EXISTS "${T001CHAIN_DIR}/cmake/general.cmake")
  message(STATUS
    "\n"
    "===========================================================\n"
    " ${T001CHAIN_DIR}\n"
    "= TOOLCHAIN_DIR is undefined                              =\n"
    "= Set this variable in your toolchain.cmake file          =\n"
    "= or verify that you called project before including      =\n"
    "= bootstrap.cmake                                         =\n"
    "===========================================================\n"
    )
  message(FATAL_ERROR "")
endif (NOT T001CHAIN_DIR OR NOT EXISTS "${T001CHAIN_DIR}/cmake/general.cmake")

include("${T001CHAIN_DIR}/cmake/general.cmake")

### Added by jayen for profiling
SET( CMAKE_CXX_FLAGS_PROFILE "-pg -O3 -DNDEBUG" CACHE STRING
    "Flags used by the C++ compiler during profile builds."
    FORCE )
SET( CMAKE_C_FLAGS_PROFILE "-pg -O3 -DNDEBUG" CACHE STRING
    "Flags used by the C compiler during profile builds."
    FORCE )
SET( CMAKE_EXE_LINKER_FLAGS_PROFILE
    "-pg" CACHE STRING
    "Flags used for linking binaries during profile builds."
    FORCE )
SET( CMAKE_MODULE_LINKER_FLAGS_PROFILE
    "-pg" CACHE STRING
    "Flags used for linking binaries during profile builds."
    FORCE )
SET( CMAKE_SHARED_LINKER_FLAGS_PROFILE
    "-pg" CACHE STRING
    "Flags used by the shared libraries linker during profile builds."
    FORCE )
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_PROFILE
    CMAKE_C_FLAGS_PROFILE
    CMAKE_EXE_LINKER_FLAGS_PROFILE
    CMAKE_MODULE_LINKER_FLAGS_PROFILE
    CMAKE_SHARED_LINKER_FLAGS_PROFILE )
SET( CMAKE_CXX_FLAGS_COVERAGE "--coverage -O3 -DNDEBUG" CACHE STRING
    "Flags used by the C++ compiler during coverage builds."
    FORCE )
SET( CMAKE_C_FLAGS_COVERAGE "--coverage -O3 -DNDEBUG" CACHE STRING
    "Flags used by the C compiler during coverage builds."
    FORCE )
SET( CMAKE_EXE_LINKER_FLAGS_COVERAGE
    "--coverage" CACHE STRING
    "Flags used for linking binaries during coverage builds."
    FORCE )
SET( CMAKE_MODULE_LINKER_FLAGS_COVERAGE
    "--coverage" CACHE STRING
    "Flags used for linking binaries during coverage builds."
    FORCE )
SET( CMAKE_SHARED_LINKER_FLAGS_COVERAGE
    "--coverage" CACHE STRING
    "Flags used by the shared libraries linker during coverage builds."
    FORCE )
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_COVERAGE
    CMAKE_C_FLAGS_COVERAGE
    CMAKE_EXE_LINKER_FLAGS_COVERAGE
    CMAKE_MODULE_LINKER_FLAGS_COVERAGE
    CMAKE_SHARED_LINKER_FLAGS_COVERAGE )
# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
### conflicts with general.cmake
#SET( CMAKE_BUILD_TYPE "" CACHE STRING
#    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Profile Coverage."
#    FORCE )
