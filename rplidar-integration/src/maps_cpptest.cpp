Copyright (c) 2009 - 2014 RoboPeak Team (http://www.robopeak.com)
Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd. (http://www.slamtec.com)
All rights reserved.

#include <maps.hpp>
#include "rplidar.h"
#include "maps_cpptest.h"


using namespace std;
using namespace rp::standalone::rplidar;

// RTMaps component for RPLidar simple object detection
class MAPSrplidar_objectdetector : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSrplidar_objectdetector)

private:
    RPlidarDriver* driver = nullptr;
};

// ---- RTMaps interface ----
MAPS_BEGIN_INPUTS_DEFINITION(MAPSrplidar_objectdetector)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSrplidar_objectdetector)
MAPS_OUTPUT("pointsOut", MAPS::Float64, nullptr, nullptr, 4096) // Output angle-distance pairs
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSrplidar_objectdetector)
MAPS_PROPERTY("lidar_ip", "192.168.1.200", false, false)
MAPS_PROPERTY("lidar_port", 2000, false, false)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSrplidar_objectdetector)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSrplidar_objectdetector, "rplidar_objectdetector", "1.0.0", 128,
    MAPS::Threaded, MAPS::Threaded,
    0,  // Nb of inputs
    1,  // Nb of outputs
    2,  // Nb of properties
    0)  // Nb of actions

    void MAPSrplidar_objectdetector::Birth()
{
    driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_TCP);
    if (!driver) {
        ReportError("Failed to create RPLidar driver");
        return;
    }
    u_result res = driver->connect(GetStringProperty("lidar_ip"), GetIntegerProperty("lidar_port"));
    if (IS_FAIL(res)) {
        ReportError("Failed to connect to RPLidar");
        RPlidarDriver::DisposeDriver(driver);
        driver = nullptr;
    }
    else {
        driver->startMotor();
        driver->startScan(false, true);
    }
}

void MAPSrplidar_objectdetector::Core()
{
    if (!driver) return;

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t node_count = 8192;

    u_result res = driver->grabScanDataHq(nodes, node_count, 200);
    if (IS_FAIL(res)) {
        ReportWarning("No scan data available.");
        Rest(100000); // 100 ms pause to prevent busy looping
        return;
    }

    driver->ascendScanData(nodes, node_count);

    // Output will be: angle1, dist1, angle2, dist2, ... up to max out buffer
    MAPSIOElt* ioElt = StartWriting(Output("pointsOut"));
    MAPSFloat64* outArray = (MAPSFloat64*)ioElt->Data();
    int max_pairs = ioElt->VectorSize() / 2;
    int n_points = min((int)node_count, max_pairs);

    for (int i = 0; i < n_points; ++i) {
        outArray[2 * i] = (nodes[i].angle_z_q14 * 90.0) / (1 << 14); // angle
        outArray[2 * i + 1] = nodes[i].dist_mm_q2 / 4.0;                 // distance
    }
    // If less than max_pairs, pad with zeros
    for (int i = n_points; i < max_pairs; ++i) {
        outArray[2 * i] = 0.0;
        outArray[2 * i + 1] = 0.0;
    }

    StopWriting(ioElt);

    Rest(100000); // 100 ms = 0.1s snapshot rate
}



void MAPSrplidar_objectdetector::Death()
{
    if (driver) {
        driver->stop();
        driver->stopMotor();
        RPlidarDriver::DisposeDriver(driver);
        driver = nullptr;
    }
}
