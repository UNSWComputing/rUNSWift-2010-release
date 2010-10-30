# Copyright (C) 2009 Aldebaran Robotics


cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
SET ( CMAKE_COLOR_MAKEFILE TRUE )
include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")

use(NAOQI-PLUGINS-TOOLS)
create_module(agent)

include("${CMAKE_CURRENT_SOURCE_DIR}/libagent.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/runswift.cmake")
