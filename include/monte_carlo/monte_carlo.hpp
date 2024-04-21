#ifndef MONTE_CARLO_H
#define MONTE_CARLO_H

#include "common/nodebase.hpp"
#include "particles.hpp"
#include "bigbang_eurobot/LaserBeacons.h"
#include "bigbang_eurobot/Move2d.h"
#include "bigbang_eurobot/MonteCarloState.h"
#include "bigbang_eurobot/Measure2d.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/PointCloud.h"
#include <QQueue>
#include <boost/unordered_map.hpp>

struct MonteCalroTopics
{
    std::string move_sub = "move_updates";
    std::string beacons_sub = "laser_beacons";
    std::string discard_service = "particles/discard_belief";
    std::string return_to_start = "particles/return_to_start";
    std::string filter_result_pub = "filtered_pos";
    std::string point_cloud_pub = "point_cloud";
    std::string filter_move_result_pub = "filtered_move";
    std::string state_pub = "monte_carlo_state";
    std::string rviz_set_point_sub = "initialpose";
    std::string child_frame_id = "filtered_pos";
    std::string frame_id = "costmap";
    std::string base_frame_id = "costmap";
};
DESCRIBE(MonteCalroTopics, 
    &_::move_sub, &_::beacons_sub, &_::discard_service,
    &_::return_to_start, &_::filter_result_pub, &_::point_cloud_pub,
    &_::filter_move_result_pub, &_::state_pub, &_::rviz_set_point_sub, 
    &_::child_frame_id,&_::frame_id,&_::base_frame_id)

struct OutputMsgsParams {
    int move_source_id = 10;
};
DESCRIBE(OutputMsgsParams, &_::move_source_id)

struct DensityParams {
    float ok_level = 0.8;
    float mid_level = 0.4;
};
DESCRIBE(DensityParams, &_::ok_level, &_::mid_level)

struct MonteCarloParams {
    DensityParams density;
    MonteCalroTopics topics;
    ParticlesSettings particles;
    OutputMsgsParams outputs;
    float max_uncertainty = 1000;
    float min_uncertainty = 0.002;
    double time_step = 0.04;
    float start_x = 1;
    float start_y = 1;
    float start_theta = 0;
    bool send_particles = true;
    bool resample_on_measure = true;
    double discard_after_bad_for = 3.5;
    cv::DecompTypes decomposition_alg = cv::DECOMP_CHOLESKY; //not-described
};

DESCRIBE(MonteCarloParams, 
    &_::density, &_::topics, &_::particles,
    &_::outputs, &_::max_uncertainty, &_::min_uncertainty,
    &_::time_step, &_::start_x, &_::start_y, 
    &_::start_theta,&_::send_particles,&_::resample_on_measure, 
    &_::discard_after_bad_for)

class MonteCarlo final : public NodeBase
{
    Q_OBJECT
public:
    MonteCarlo();
    void updateParams();
private:
    void applyMovesStep();
    void moveCb(const bigbang_eurobot::Move2dConstPtr &msg);
    void beaconsCb(const bigbang_eurobot::LaserBeaconsConstPtr &msg);
    void rvizInitialCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    MeasurePair collectMoves(bool *ok = nullptr);
    void publishState();
    bool discardBelief(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp);
    bool returnToStart(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp);

    MonteCarloParams params;
    Particles m_particles;
    QTimer *m_updateTimer;
    boost::unordered_map<quint16, MeasurePair> m_currentMoves;
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
