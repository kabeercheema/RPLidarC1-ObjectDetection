////////////////////////////////////////////////////////////////////////////////
//     This file is part of RTMaps                                            //
//     Copyright (c) Intempora S.A. All rights reserved.                      //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////
// SDK Programmer samples
////////////////////////////////

#pragma once

// Includes maps sdk library header
#include <maps.hpp>
// Includes the MAPS::InputReader class and its dependencies
#include <maps/input_reader/maps_input_reader.hpp>
#include <iostream>
#include <rplidar.h>
#include <numeric>
#include <algorithm>
#include <vector>
#include <cmath>

using namespace rp::standalone::rplidar;
// Declares a new MAPSComponent child class
class MAPSmy_rplidarc1 : public MAPSComponent
{
    // Use standard header definition macro
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSmy_rplidarc1)
private:
    RPlidarDriver* driver;
    // Place here your specific methods and attributes
};