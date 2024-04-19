#include "bigbang/nodes/rplidarnode.h"
#include "bigbang_eurobot/MapObject.h"
#include "pcg/pcg_random.hpp"
#include "qthread.h"
#include <boost/filesystem.hpp>
#include <math.h>
#include <random>

using namespace sl;
namespace fs = boost::filesystem;
using ms = std::chrono::milliseconds;
using namespace std::chrono;
using sys_clock = std::chrono::high_resolution_clock;

const QString &LidarNode::baseFrameId() const
{
    return m_params.topics.base_frame_id;
}

LidarNode::LidarNode(const NodeSettings &settings) :
    NodeBase(settings),
    m_driver(*createLidarDriver()),
    m_mainLoop(new QTimer(this))
{
    m_mainLoop->callOnTimeout(this, &LidarNode::mainLoop);
    int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
    int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
    int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;    
    ROS_INFO("Starting Lidar Node... (SDK Version:%d.%d.%d)", ver_major, ver_minor, ver_patch);
    m_monteCarloState.is_bad = true;
    updateParams();
}

LidarNode::~LidarNode()
{
    qDebug("Lidar shutting down...");
    ROS_INFO("Lidar shutting down...");
    stopMotor();
}

void LidarNode::monteCarloStateCb(const bigbang_eurobot::MonteCarloStateConstPtr &state)
{
    m_monteCarloState = *state;
}

void LidarNode::updateParams(const QString &name)
{
    m_params.update(name);
    m_nodes.resize(MAX_NODES);
    m_parsedNodes.resize(MAX_NODES);
    m_scanMsg.ranges.resize(MAX_NODES);
    m_posOffset = {m_params.lidar_x_offset, m_params.lidar_y_offset};
    m_scanMsg.intensities.resize(MAX_NODES);
    m_scanMsg.range_min = m_params.range_min;
    m_scanMsg.range_max = m_params.range_max;
    m_tfMsg.header.frame_id = m_params.topics.frame_id.toStdString();
    m_tfMsg.child_frame_id = m_params.topics.child_frame_id.toStdString();
    m_tfMsg.transform.translation.x = m_params.lidar_x_offset;
    m_tfMsg.transform.translation.y = m_params.lidar_y_offset;
    m_tfMsg.transform.rotation = msgFromTheta(m_params.lidar_offset);
    m_scanMsg.header.frame_id = m_params.topics.child_frame_id.toStdString().c_str();
    m_posSub = nh()->subscribe(m_params.topics.position_sub.toStdString(), 20, &LidarNode::positionCb, this);
    m_moveSub = nh()->subscribe(m_params.topics.move_sub.toStdString(), 20, &LidarNode::moveCb, this);
    m_monteCarloSub = nh()->subscribe(m_params.topics.monte_carlo_sub.toStdString(), 20, &LidarNode::monteCarloStateCb, this);
    m_objsPub = nh()->advertise<bigbang_eurobot::MapObject>(m_params.topics.objects_pub.toStdString(), 700);
    m_scanPub = nh()->advertise<sensor_msgs::LaserScan>(m_params.topics.laser_pub.toStdString(), 20);
    m_beaconsPub = nh()->advertise<bigbang_eurobot::LaserBeacons>(m_params.topics.beacons_pub.toStdString(), 20);
    m_closestObjPub = nh()->advertise<bigbang_eurobot::MapObject>(m_params.topics.closest_obj_pub.toStdString(), 20);
    if (!m_params.test_mode) {
        if (!m_driver->isConnected()) {
            connectDriver();
            if (!getDeviceInfo()) {
                ROS_ERROR_STREAM("Unable to get device info!");
                shutdown();
            }
            if (!checkHealth()) {
                ROS_ERROR_STREAM("Health check error!");
                shutdown();
            }
            if (!initMode()) {
                ROS_ERROR_STREAM("Unsupported mode: " + m_params.scan_mode.toStdString());
                shutdown();
            }
            startMotor();
        }
    }
    initBeacons();
    initDetection();
    m_mainLoop->start(0);
}

void LidarNode::initBeacons()
{
    auto beaconsCount = m_params.beacons.all_poses_x.size();
    if (m_params.beacons.all_poses_y.size() != beaconsCount || m_params.beacons.all_shimmering_hz.size() != beaconsCount) {
        ROS_ERROR("Inconsistent beacons params (all arrays must be same size!)");
    }
    m_beacons.reserve(beaconsCount);
    for (auto i = 0; i < beaconsCount; i++) {
        auto currentBeacon = LaserBeacon(
                    m_params.beacons.all_shimmering_hz[i],
                   {m_params.beacons.all_poses_x[i], m_params.beacons.all_poses_y[i]},
                    i);
        m_beacons.push_back(currentBeacon);
    }
    m_shape = {m_beacons, m_params.beacons};
    m_beaconsMsg.beacons.reserve(m_beacons.size());
}

void LidarNode::initDetection()
{
    m_baseOffset = {1, 0};
    m_baseOffset = m_baseOffset.rotated_by(m_params.lidar_offset);
}

void LidarNode::startMotor()
{
    if (!m_needsTune) {
        m_driver->setMotorSpeed(m_params.scan_frequency * 60);
        ROS_INFO("set lidar scan frequency to %.1f Hz(%.1f Rpm) ", m_params.scan_frequency, m_params.scan_frequency*60);
    }
}

void LidarNode::stopMotor()
{
    m_driver->setMotorSpeed(0);
    ROS_INFO("Motor stopped");
    qDebug("Motor stopped");
}

void LidarNode::grabMock(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count)
{
    count = 720;
    auto noize = 0.005;
    auto sleepTime = 0.1;
    pcg32_fast e2(std::random_device{}());
    std::uniform_real_distribution<float> distr(-noize, noize);
    auto rand = [&]() mutable {
        return distr(e2);
    };
    auto coeff = (360.f / count) / (90.f / 16384.f);
    auto fillAll = [&]() mutable {
        auto dist = 0.5;
        for (int i = 0; i < count; i++) {
            nodebuffer[i] = sl_lidar_response_measurement_node_hq_t {
                static_cast<sl_u16>(i * coeff * (m_params.reversed?-1:1)),
                static_cast<sl_u32>((dist + rand()) * 4000 * 1000),
                47 << 2,
                0
            };
        }
    };
    auto spawnGroup = [&](int start, int end, float dist) mutable {
        for (int i = start; i < end; i++) {
            nodebuffer[i] = sl_lidar_response_measurement_node_hq_t {
                static_cast<sl_u16>(i * coeff * (m_params.reversed?-1:1)),
                static_cast<sl_u32>((dist + rand()) * 4000),
                47 << 2,
                0
            };
        }
    };
    fillAll();
    spawnGroup(98, 108, 1.58);
    spawnGroup(168, 188, 1.32);
    spawnGroup(375, 395, 1.03);
    spawnGroup(595, 605, 1.95);
    spawnGroup(200, 210, 2);
    spawnGroup(220, 230, 2.1);
    spawnGroup(240, 250, 2);
    if (m_params.reversed) {
        for (int i = 0; i < count/2; i++) {
            std::swap(nodebuffer[i], nodebuffer[count - i - 1]);
        }
    }
    QThread::msleep(sleepTime * 1000);
}


void LidarNode::mainLoop()
{
    auto count = MAX_NODES;
    auto startTime = sys_clock::now();
    if (m_params.test_mode) {
        grabMock(m_nodes.data(), count);
    } else {
        sl_result grabResult;
        if (m_params.grab_with_interval) {
            grabResult = m_driver->getScanDataWithIntervalHq(m_nodes.data(), count);
        } else {
            grabResult = m_driver->grabScanDataHq(m_nodes.data(), count);
        }
        if (!isOk(grabResult)) {
            ROS_ERROR_STREAM("Lidar Grab Scan Error! Reason: " << printResult(grabResult));
            return;
        }
        if (tuneRpmAfterScan()) return;
        auto asc_res = m_driver->ascendScanData(m_nodes.data(), count);
        if (!isOk(asc_res)) {
            ROS_ERROR_STREAM("Lidar Ascend Data Error! Reason: " << printResult(asc_res));
            return;
        }
    }
    m_nodes.resize(count);
    m_scanMsg.ranges.resize(count);
    m_scanMsg.intensities.resize(count);
    m_parsedNodes.resize(count);
    auto passedTime = duration_cast<ms>(sys_clock::now() - startTime).count() / 1000.f;
    m_scanMsg.angle_min = toRadians(0);
    m_scanMsg.angle_max = toRadians(360);
    m_scanMsg.angle_increment = toRadians(360) / count;
    m_scanMsg.header.seq++;
    m_scanMsg.scan_time = passedTime;
    m_scanMsg.header.stamp = ros::Time::now();
    m_scanMsg.time_increment = passedTime / count;
    if (m_params.reversed) {
        std::reverse(m_nodes.begin(), m_nodes.end());
    }
    for (size_t i = 0; i < count; i++) {
        m_parsedNodes[i].intensity = static_cast<float>(m_nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        m_parsedNodes[i].range = static_cast<float>(m_nodes[i].dist_mm_q2/4000.f) + m_params.range_correction;
        if (m_parsedNodes[i].range < m_params.range_min) {
            m_parsedNodes[i].range = std::numeric_limits<float>::infinity();
        }
        m_parsedNodes[i].relTheta = toRadians(m_nodes[i].angle_z_q14 * 90.f / 16384.f) * (m_params.reversed? -1 : 1);
        auto norm = m_baseOffset.rotated_by(m_parsedNodes[i].relTheta + m_position.getTheta());
        m_parsedNodes[i].position = norm * m_parsedNodes[i].range + m_position.toCoord() + m_posOffset;
        m_scanMsg.intensities[i] = m_parsedNodes[i].intensity;
        m_scanMsg.ranges[i] = m_parsedNodes[i].range;
    }
    auto nowTs = sys_clock::now();
    auto passedTimeMs = duration_cast<ms>(nowTs - m_lastUpdate).count();
    m_lastUpdate = nowTs;
    if (m_monteCarloState.is_ok) {
        m_timeMonteOk += passedTimeMs/1000.f;
        m_timeMonteBad = 0;
    } else {
        m_timeMonteOk = 0;
        m_timeMonteBad += passedTimeMs/1000.f;
    }
    detectObjects();
    m_objects.update(passedTimeMs);
    detectBeacons(passedTimeMs);
    sendObjects();
    visualize();
    if (m_params.send_scans) {
        m_scanPub.publish(m_scanMsg);
        m_tfMsg.header.stamp = m_scanMsg.header.stamp;
        m_tfMsg.header.seq++;
        m_tfBroad.sendTransform(m_tfMsg);
    }
}

void LidarNode::detectObjects() {
    auto validMinPos = Coord_<float>(m_params.objects.min_x, m_params.objects.min_y);
    auto validMaxPos = Coord_<float>(m_params.objects.max_x, m_params.objects.max_y);
    auto currentStart = -1;
    auto wasCloseEnough = false;
    for (quint32 i = 1; i < m_parsedNodes.size(); ++i) {
        auto &last = m_parsedNodes[i - 1];
        auto &current = m_parsedNodes[i];
        auto lastValid = m_position.dist_to(last.position) < m_params.objects.max_dist;
        auto currentValid = m_position.dist_to(current.position) < m_params.objects.max_dist;
        auto dist = current.position.dist_to(last.position);
        auto closeEnough = dist <= m_params.objects.max_dist_between_dots;
        if (lastValid && currentStart == -1 && closeEnough) {
            currentStart = i - 1;
        }
        auto jumped = (!closeEnough && wasCloseEnough) || !currentValid;
        auto startValid = currentStart != -1;
        auto dotsDiff = i - currentStart + 1;
        auto tooMany = dotsDiff > m_params.objects.split_each;
        wasCloseEnough = closeEnough;
        if ((jumped || tooMany || i == m_parsedNodes.size() - 1) && startValid) {
            auto enoughDots = dotsDiff >= m_params.objects.min_points;
            if (!enoughDots) {
                currentStart = -1;
                continue;
            }
            auto startPos = m_parsedNodes[currentStart].position;
            auto stopPos = m_parsedNodes[i - 1].position;
            auto middle = startPos + (stopPos - startPos) / 2;
            auto size = stopPos.dist_to(startPos);
            auto correction = CoordF(middle - m_position.toCoord()).normalized() * size / 1.9f;
            LaserObject currentObj(middle + correction, size, m_params.objects.start_ttl, m_position.dist_to(middle));
            if (m_position.dist_to(currentObj.position) <= m_params.objects.max_dist &&
                    currentObj.posValid(validMinPos, validMaxPos)) {
                m_objects.insertFound(currentObj, m_params.objects);
            }
            currentStart = -1;
        }
    }
}

void LidarNode::detectBeacons(float passedTimeMs)
{
    if (!m_params.enable_beacons) return;
    if (m_timeMonteBad > m_params.beacons.time_before_global) {
        m_shouldGlobalLocalize = true;
    } else if (m_timeMonteOk > m_params.beacons.time_for_global) {
        m_shouldGlobalLocalize = false;
    }
    m_beaconsMsg.beacons.clear();
    for (auto &beacon: m_beacons) {
        beacon.setLost(passedTimeMs);
    }
    auto testSize = [&](const LaserObject *obj) {
        return m_params.beacons.min_size <= obj->size && obj->size <= m_params.beacons.max_size;
    };
    auto useSimple = !m_shouldGlobalLocalize;
    if (useSimple) {
        ROS_INFO_THROTTLE(3, "[Lidar] Using SIMPLE localization");
        // while ok --> minor correction using closest beacons
        for (auto &beacon: m_beacons) {
            auto closest = m_objects.closestByPos(beacon.targetPos, m_params.beacons.simple.dist_margin);
            if (closest && testSize(closest)) {
                beacon.setFound(*closest);
                m_beaconsMsg.beacons.push_back(beacon.toMsg(m_position));
            }
        }
    } else {
        ROS_INFO_THROTTLE(1.5, "[Lidar] Using GLOBAL localization");
        auto result = m_shape.findIn(m_objects);
        for (int i = 0; i < result.size(); ++i) {
            auto &obj = result[i];
            auto &beacon = m_beacons[i];
            if (obj) {
                beacon.setFound(*obj);
                m_beaconsMsg.beacons.push_back(beacon.toMsg(m_position));
            }
        }
    }
    m_beaconsPub.publish(m_beaconsMsg);
}

void LidarNode::sendObjects()
{
    bigbang_eurobot::MapObject closest;
    float minDist = std::numeric_limits<float>::max();
    for (const auto& obj: m_objects) {
        auto toSend = bigbang_eurobot::MapObject();
        toSend.id = obj.id;
        toSend.size = obj.size;
        toSend.ttl = obj.ttl * m_params.objects.map_ttl_coeff;
        toSend.x = obj.position.getX();
        toSend.y = obj.position.getY();
        toSend.dist_to = obj.distTo;
        if (toSend.dist_to < minDist) {
            closest = toSend;
            minDist = toSend.dist_to;
        }
        m_objsPub.publish(toSend);
    }
    if (m_objects.size()) {
        m_closestObjPub.publish(closest);
    }
}


void LidarNode::visualize()
{
    for (auto &beacon: m_beacons) {
        drawSphere(beacon.targetPos, m_scanMsg.scan_time * 1.1, m_scanMsg.scan_time * 2, 1, 1, 1, 0.3);
        if (beacon.isValid) {
            drawSphere(beacon.realPos, beacon.size, 0.2);
        }
    }
}

void LidarNode::connectDriver()
{
    ROS_WARN("Connecting to Lidar");
    if (m_channel) {
        delete m_channel;
        m_channel = nullptr;
    }
    if (m_params.use_serial) {
        fs::path port(m_params.serial.port.toStdString());
        if (!fs::exists(port)) {
            ROS_ERROR_STREAM("Port does not exist: " << port.generic_string());
            shutdown();
        }
        m_channel = *createSerialPortChannel(m_params.serial.port.toStdString(), m_params.serial.baudrate);
    } else if (m_params.network.use_tcp) {
        m_channel = *createTcpChannel(m_params.network.host.toStdString(), m_params.network.port);
    } else {
        m_channel = *createUdpChannel(m_params.network.host.toStdString(), m_params.network.port);
    }
    if (!m_channel) {
        ROS_ERROR_STREAM("Error connecting to lidar");
        shutdown();
    }
    auto result = m_driver->connect(m_channel);
    if (!isOk(result)) {
        ROS_ERROR_STREAM("Lidar Connection Error: " + printResult(result));
        ROS_ERROR_STREAM("Errno: " << Helper::printErrno().toStdString());
        ROS_ERROR_STREAM("BAUD: " << m_params.serial.baudrate);
        shutdown();
    } else {
        ROS_INFO_STREAM("Connection status: " << printResult(result));
    }
}

void LidarNode::positionCb(const bigbang_eurobot::Measure2dConstPtr &pos)
{
    m_position.setX(pos->x);
    m_position.setY(pos->y);
    m_position.setTheta(pos->theta);
}

void LidarNode::moveCb(const bigbang_eurobot::Move2dConstPtr &move)
{
    m_move = *move;
}

bool LidarNode::checkHealth()
{
    sl_lidar_response_device_health_t healthinfo;
    auto op_result = m_driver->getHealth(healthinfo);
    if (isOk(op_result)) { 
        switch (healthinfo.status) {
			case SL_LIDAR_STATUS_OK:
                ROS_INFO("RPLidar health status : OK.");
				return true;
			case SL_LIDAR_STATUS_WARNING:
                ROS_INFO("RPLidar health status : Warning.");
				return true;
			case SL_LIDAR_STATUS_ERROR:
                ROS_ERROR("Error, rplidar internal error detected. Please reboot the device to retry.");
				return false;
            default:
                ROS_ERROR("Unkown Error");
                return false;
        }
    } else {
        ROS_ERROR_STREAM("Error, cannot retrieve rplidar health code: " + printResult(op_result));
        shutdown();
        return false;
    }
    return true;
}
bool LidarNode::getDeviceInfo()
{
    sl_result     op_result;
    sl_lidar_response_device_info_t devinfo;
    op_result = m_driver->getDeviceInfo(devinfo);
    if (!isOk(op_result)) {
        ROS_ERROR_STREAM("Error, unexpected error, code: " << printResult(op_result));
        ROS_ERROR_STREAM("Errno: " << Helper::printErrno().toStdString());
        return false;
    }
    char sn_str[35] = {0}; 
    for (int pos = 0; pos < 16 ;++pos) {
        sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
    }
    char mode_str[16] = {0};
    m_needsTune = true;
    if((devinfo.model>>4) <= LIDAR_S_SERIES_MINUM_MAJOR_ID){
        sprintf(mode_str,"A%dM%d",(devinfo.model>>4),(devinfo.model&0xf));
        m_modelFamily = SeriesA;
        m_needsTune = false;
        m_driver->setMotorSpeed(600);
    }else if((devinfo.model>>4) <= LIDAR_T_SERIES_MINUM_MAJOR_ID){
        sprintf(mode_str,"S%dM%d",(devinfo.model>>4)-LIDAR_S_SERIES_MINUM_MAJOR_ID,(devinfo.model&0xf));
        m_modelFamily = SeriesS;
    }else{
        sprintf(mode_str,"T%dM%d",(devinfo.model>>4)-LIDAR_T_SERIES_MINUM_MAJOR_ID,(devinfo.model&0xf));
        m_modelFamily = SeriesT;
    }
    ROS_INFO("RPLIDAR MODE: %s",mode_str);
    ROS_INFO("RPLIDAR S/N: %s",sn_str);
    ROS_INFO("Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    ROS_INFO("Hardware Rev: %d",(int)devinfo.hardware_version);
    ROS_INFO_STREAM("RPLIDAR series: " << QMetaEnum::fromType<ModelFamily>().valueToKey(m_modelFamily));
    return true;
}

bool LidarNode::initMode()
{
    std::vector<LidarScanMode> allSupportedScanModes;
    m_driver->getAllSupportedScanModes(allSupportedScanModes);
    bool found = false;
    for (auto &mode : allSupportedScanModes) {
        ROS_WARN("Supported Mode: %s: max_distance: %.1f m, Point number: %.1fK", 
        mode.scan_mode, mode.max_distance, (1000/mode.us_per_sample));
        if (mode.scan_mode == m_params.scan_mode) {
            m_lidarMode = mode;   
            m_driver->startScan(false, m_lidarMode.id, 0, &m_lidarMode);
            found = true;
            ROS_WARN_STREAM("Using Mode: " << mode.scan_mode);
        }
    }
    return found;
}

bool LidarNode::tuneRpmAfterScan()
{
    if (m_needsTune) {
        ROS_INFO("[TUNE] set lidar scan frequency to %.1f Hz(%.1f Rpm) ", m_params.scan_frequency, m_params.scan_frequency*60);
        m_driver->setMotorSpeed(m_params.scan_frequency * 60); //rpm
        m_needsTune = false;
        return true;
    }
    return false;
}

std::string LidarNode::printResult(Results result) const
{
    return QMetaEnum::fromType<Results>().valueToKey(result);
}

std::string LidarNode::printResult(sl_result result) const
{
    return printResult(static_cast<Results>(result));
}

NODE_MAIN(LidarNode, "bigbang_rplidar")
