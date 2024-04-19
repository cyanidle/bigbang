#ifndef MONTE_CARLO_H
#define MONTE_CARLO_H

#include "bigbang/nodes/nodebase.h"
#include "bigbang/utils/particles.hpp"
#include "bigbang_eurobot/LaserBeacons.h"
#include "bigbang_eurobot/Move2d.h"
#include "bigbang_eurobot/MonteCarloState.h"
#include "bigbang_eurobot/Measure2d.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/PointCloud.h"
#include <QQueue>

struct MonteCalroTopics : public RosParams
{
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(QString, move_sub, "move_updates")
    SERIAL_FIELD(QString, beacons_sub, "laser_beacons")
    SERIAL_FIELD(QString, discard_service, "particles/discard_belief")
    SERIAL_FIELD(QString, return_to_start, "particles/return_to_start")
    SERIAL_FIELD(QString, filter_result_pub, "filtered_pos")
    SERIAL_FIELD(QString, point_cloud_pub, "point_cloud")
    SERIAL_FIELD(QString, filter_move_result_pub, "filtered_move")
    SERIAL_FIELD(QString, state_pub, "monte_carlo_state")
    SERIAL_FIELD(QString, rviz_set_point_sub, "initialpose")
    SERIAL_FIELD(QString, child_frame_id, "filtered_pos")
    SERIAL_FIELD(QString, frame_id, "costmap")
    SERIAL_FIELD(QString, base_frame_id, "costmap")
};

struct OutputMsgsParams : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(int, move_source_id, 10)
};

struct DensityParams : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(float, ok_level, 0.8)
    SERIAL_FIELD(float, mid_level, 0.4)
};

struct MonteCarloParams : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_NEST(DensityParams, density, DEFAULT)
    SERIAL_NEST(MonteCalroTopics, topics, DEFAULT)
    SERIAL_NEST(ParticlesSettings, particles, DEFAULT)
    SERIAL_NEST(OutputMsgsParams, outputs, DEFAULT)
    SERIAL_FIELD(float, max_uncertainty, 1000)
    SERIAL_FIELD(float, min_uncertainty, 0.002)
    SERIAL_FIELD(double, time_step, 0.04)
    SERIAL_FIELD(float, start_x, 1)
    SERIAL_FIELD(float, start_y, 1)
    SERIAL_FIELD(float, start_theta, 0)
    SERIAL_FIELD(bool, send_particles, true)
    SERIAL_FIELD(bool, resample_on_measure, true)
    SERIAL_FIELD(double, discard_after_bad_for, 3.5)
    SERIAL_FIELD(QString, decomposition_alg, "cholesky")
    cv::DecompTypes decomposition_alg_{cv::DECOMP_CHOLESKY};

    SERIAL_POST_INIT(prvInit)
    void prvInit();
};

class MonteCarlo : public NodeBase
{
    Q_OBJECT
public:
    MonteCarlo(const NodeSettings &settings);
    void updateParams(const QString &name = "") override final;
    const QString &baseFrameId() const override final;
private:
    void applyMovesStep();
    void moveCb(const bigbang_eurobot::Move2dConstPtr &msg);
    void beaconsCb(const bigbang_eurobot::LaserBeaconsConstPtr &msg);
    void rvizInitialCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    MeasurePair collectMoves(bool *ok = nullptr);
    void publishState();
    bool discardBelief(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp);
    bool returnToStart(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp);

    MonteCarloParams m_params;
    Particles m_particles;
    QTimer *m_updateTimer;
    QHash<quint16, MeasurePair> m_currentMoves;
    ros::Subscriber m_moveSub;
    ros::Subscriber m_measureSub;
    ros::Subscriber m_rvizInitialSub;
    ros::Publisher m_resultPub;
    ros::Publisher m_moveResultPub;
    ros::Publisher m_pointCloudPub;
    ros::Publisher m_statePub;
    ros::ServiceServer m_discardBelief;
    ros::ServiceServer m_returnToStart;
    tf::TransformBroadcaster m_tfBroad;
    geometry_msgs::TransformStamped m_tfMsg;
    bigbang_eurobot::Measure2d m_positionMsg;
    bigbang_eurobot::Move2d m_moveResultMsg;
    sensor_msgs::PointCloud m_pointCloud;
    bigbang_eurobot::MonteCarloState m_state;
    float m_badStateFor{0};
};

#endif
