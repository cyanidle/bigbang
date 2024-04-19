#ifndef LIDAR_HANDLER_H
#define LIDAR_HANDLER_H
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <ros/ros.h>
#include "bigbang/types/costmap.hpp"
#include "bigbang/utils/beacons_shape.hpp"
#include "bigbang_eurobot/Measure2d.h"
#include "bigbang_eurobot/MonteCarloState.h"
#include "bigbang_eurobot/Move2d.h"
#include "nodebase.h"
#include "bigbang/types/rosparams.h"
#include "tf/transform_broadcaster.h"
#include "sl_lidar.h"
#include "sensor_msgs/LaserScan.h"
#include "bigbang/utils/rplidar_utils.h"
#include <boost/pool/pool_alloc.hpp>
#include <chrono>
#include <bigbang_eurobot/LaserBeacons.h>

#define MAX_NODES 8192UL

struct LidarTopics : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(QString, base_frame_id, "costmap")
    SERIAL_FIELD(QString, frame_id, "filtered_pos")
    SERIAL_FIELD(QString, child_frame_id, "laser")
    SERIAL_FIELD(QString, position_sub, "filtered_pos")
    SERIAL_FIELD(QString, monte_carlo_sub, "monte_carlo_state")
    SERIAL_FIELD(QString, move_sub, "filtered_move")
    SERIAL_FIELD(QString, laser_pub, "scan")
    SERIAL_FIELD(QString, objects_pub, "obstacles")
    SERIAL_FIELD(QString, beacons_pub, "laser_beacons")
    SERIAL_FIELD(QString, closest_obj_pub, "closest_object")
};

struct NetworkParams : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(QString, host, "192.168.1.25")
    SERIAL_FIELD(int, port, 20108)
    SERIAL_FIELD(bool, use_tcp, true)

};

struct SerialParams : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(int, baudrate, 256400)
    SERIAL_FIELD(QString, port, "/dev/ttyUSB0")
};

struct LidarParams : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(bool, test_mode, false)
    SERIAL_FIELD(bool, enable_beacons, true)
    SERIAL_FIELD(bool, send_scans, true)
    SERIAL_FIELD(bool, grab_with_interval, false)
    SERIAL_FIELD(bool, use_serial, true)
    SERIAL_FIELD(bool, reversed, true) // if scan comes clockwise - it is reversed
    SERIAL_FIELD(double, scan_frequency, 10)
    SERIAL_FIELD(float, range_min, 0.15)
    SERIAL_FIELD(float, range_max, 40)
    SERIAL_FIELD(float, lidar_offset, 0)
    SERIAL_FIELD(float, lidar_x_offset, 0)
    SERIAL_FIELD(float, lidar_y_offset, 0)
    SERIAL_FIELD(float, range_correction, 0)
    SERIAL_FIELD(int, source_id, 0)
    SERIAL_FIELD(QString, scan_mode, "DenseBoost")
    SERIAL_NEST(ObjectDetection, objects, DEFAULT)
    SERIAL_NEST(BeaconsParams, beacons, DEFAULT)
    SERIAL_NEST(NetworkParams, network, DEFAULT)
    SERIAL_NEST(SerialParams, serial, DEFAULT)
    SERIAL_NEST(LidarTopics, topics, DEFAULT)
};

class LidarNode : public NodeBase
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
    const QString &baseFrameId() const override;
    LidarNode(const NodeSettings &settings);
    ~LidarNode();
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
    void updateParams(const QString &name = "") override final;
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
    LidarParams m_params{};
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

#endif
