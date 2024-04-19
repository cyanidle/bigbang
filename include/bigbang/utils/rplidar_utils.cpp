#include "rplidar_utils.h"

float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

bool isOk(sl_result result)
{
    return SL_IS_OK(result);
}
