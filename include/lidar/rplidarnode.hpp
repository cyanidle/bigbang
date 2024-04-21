#pragma once

#include <QTimer>
#include <chrono>
#include <ros/ros.h>
#include <boost/pool/pool_alloc.hpp>
#include <tf/transform_broadcaster.h>

#include "bigbang_eurobot/Measure2d.h"
#include "bigbang_eurobot/MonteCarloState.h"
#include "bigbang_eurobot/Move2d.h"
#include "bigbang_eurobot/LaserBeacons.h"

#include "sl_lidar.h"

#include "common/nodebase.hpp"
#include "common/costmap.hpp"
#include "beacons_shape.hpp"
#include "sensor_msgs/LaserScan.h"
#include "describe/describe.hpp"
#include "rplidar_utils.hpp"

#define MAX_NODES 8192UL

struct LidarTopics {
    std::string base_frame_id = {"costmap"};
    std::string frame_id = {"filtered_pos"};
    std::string child_frame_id = {"laser"};
    std::string position_sub = {"filtered_pos"};
    std::string monte_carlo_sub = {"monte_carlo_state"};
    std::string move_sub = {"filtered_move"};
    std::string laser_pub = {"scan"};
    std::string objects_pub = {"obstacles"};
    std::string beacons_pub = {"laser_beacons"};
    std::string closest_obj_pub = {"closest_object"};
};
DESCRIBE(LidarTopics, 
    &_::base_frame_id, &_::frame_id, &_::child_frame_id,
    &_::position_sub, &_::monte_carlo_sub, &_::move_sub,
    &_::laser_pub, &_::objects_pub, &_::beacons_pub, &_::closest_obj_pub)

struct NetworkParams {
    std::string host = "192.168.1.25";
    int port = 20108;
    bool use_tcp = true;
};
DESCRIBE(NetworkParams, &_::host, &_::port, &_::use_tcp)

struct SerialParams {
    int baudrate = 256400;
    std::string port = "/dev/ttyUSB0";
};
DESCRIBE(SerialParams, &_::baudrate, &_::port)

struct LidarParams {
    bool test_mode = false;
    bool enable_beacons = true;
    bool send_scans = true;
    bool grab_with_interval = false;
    bool use_serial = true;
    bool reversed = true; // if scan comes clockwise - it i = reverse;
    double scan_frequency = 10;
    float range_min = 0.15;
    float range_max = 40;
    float lidar_offset = 0;
    float lidar_x_offset = 0;
    float lidar_y_offset = 0;
    float range_correction = 0;
    int source_id = 0;
    std::string scan_mode = "DenseBoost";
    ObjectDetection objects;
    BeaconsParams beacons;
    NetworkParams network;
    SerialParams serial;
    LidarTopics topics;
};
DESCRIBE(LidarParams, 
    &_::test_mode, &_::enable_beacons, &_::send_scans,
    &_::grab_with_interval, &_::use_serial, &_::reversed,
    &_::scan_frequency, &_::range_min, &_::range_max, &_::lidar_offset,
    &_::lidar_x_offset,&_::lidar_y_offset,&_::range_correction,&_::source_id,
    &_::scan_mode,&_::objects,&_::beacons,&_::network,&_::serial,&_::topics)

class LidarNode final : public NodeBase
{
    Q_OBJECT
public:
    using Node = sl_lidar_response_measurement_node_hq_t; 
    enum MinimumIds {
        LIDAR_A_SERIES_MINUM_MAJOR_ID = 0,
        LIDAR_S_SERIES_MINUM_MAJOR_ID = 5,
        LIDAR_T_SERIES_MINUM_MAJOR_ID = 8,
    };
    enum Results : quint64 {
        ResultOk = 0,
        ResultAlreadyDone = 0x20,
        ResultInvalidData = (0x8000 | SL_RESULT_FAIL_BIT),
        ResultOperationFail = (0x8001 | SL_RESULT_FAIL_BIT),
        ResultOperationTimeout = (0x8002 | SL_RESULT_FAIL_BIT),
        ResultOperationStop = (0x8003 | SL_RESULT_FAIL_BIT),
        ResultOperationNotSupported = (0x8004 | SL_RESULT_FAIL_BIT),
        ResultFormatNotSupported = (0x8005 | SL_RESULT_FAIL_BIT),
        ResultInsufficientMemory = (0x8006 | SL_RESULT_FAIL_BIT),
    };
    enum ModelFamily {
        Unkown = 0,
        SeriesA,
        SeriesS,
        SeriesT,
    };
    Q_ENUM(ModelFamily)
    Q_ENUM(Results)

    LidarNode();
    ~LidarNode() override;
signals:
    void beaconFindFailed();
private slots:
    void stopMotor();
    void startMotor();
    void mainLoop();
    void connectDriver();
private:
    void positionCb(const bigbang_eurobot::Measure2dConstPtr &pos);
    void moveCb(const bigbang_eurobot::Move2dConstPtr &move);
    void monteCarloStateCb(const bigbang_eurobot::MonteCarloStateConstPtr &state);
    void updateParams();
    bool checkHealth();
    bool getDeviceInfo();
    bool tuneRpmAfterScan();
    bool initMode();
    void initBeacons();
    void initDetection();
    void detectObjects();
    void detectBeacons(float passedTimeMs);
    void visualize();
    void grabMock(sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &count);
    void sendObjects();

    std::string printResult(Results result) const;
    std::string printResult(sl_result result) const;

    sl::ILidarDriver *m_driver{nullptr};
    sl::IChannel *m_channel{nullptr};
    QTimer *m_mainLoop{nullptr};

    std::vector<Node> m_nodes{};
    std::vector<ParsedNode> m_parsedNodes{};
    ObjectsMap m_objects{};

    CoordF m_baseOffset{};
    CoordF m_posOffset{};
    PositionF m_position{};

    sl::LidarScanMode m_lidarMode{};
    sensor_msgs::LaserScan m_scanMsg{};
    LidarParams params{};
    bool m_needsTune{false};
    ros::Subscriber m_posSub{};
    ros::Subscriber m_moveSub{};
    ros::Subscriber m_monteCarloSub{};
    ros::Publisher m_scanPub{};
    ros::Publisher m_objsPub{};
    ros::Publisher m_beaconsPub{};
    ros::Publisher m_closestObjPub{};
    bigbang_eurobot::Move2d m_move{};
    ModelFamily m_modelFamily;
    tf::TransformBroadcaster m_tfBroad{};
    geometry_msgs::TransformStamped m_tfMsg{};
    bigbang_eurobot::MonteCarloState m_monteCarloState;
    std::vector<LaserBeacon> m_beacons;
    bigbang_eurobot::LaserBeacons m_beaconsMsg;
    BeaconsShape m_shape;
    std::chrono::system_clock::time_point m_lastUpdate;

    float m_timeMonteOk{0};
    float m_timeMonteBad{0};
    bool m_shouldGlobalLocalize{false};
};
