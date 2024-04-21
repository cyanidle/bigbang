#include "monte_carlo/monte_carlo.hpp"
#include "common/nodebase.hpp"

#define MAX_UNC params.max_uncertainty
#define MAX_UNC_9   MAX_UNC, 0, 0, \
                    0, MAX_UNC, 0, \
                    0, 0, MAX_UNC

MonteCarlo::MonteCarlo() :
    params(),
    m_particles(params.particles),
    m_updateTimer(new QTimer(this))
{
    updateParams();
    m_updateTimer->callOnTimeout(this, &MonteCarlo::applyMovesStep);
}

void MonteCarlo::updateParams()
{
    DeserializeFromRos(params);
    m_particles = Particles{params.particles};
    m_moveSub = nh()->subscribe(params.topics.move_sub, 20, &MonteCarlo::moveCb, this);
    m_measureSub = nh()->subscribe(params.topics.beacons_sub, 20, &MonteCarlo::beaconsCb, this);
    m_rvizInitialSub = nh()->subscribe(params.topics.rviz_set_point_sub, 20, &MonteCarlo::rvizInitialCb, this);
    m_resultPub = nh()->advertise<bigbang_eurobot::Measure2d>(params.topics.filter_result_pub, 20);
    m_pointCloudPub = nh()->advertise<sensor_msgs::PointCloud>(params.topics.point_cloud_pub, 20);
    m_moveResultPub = nh()->advertise<bigbang_eurobot::Move2d>(params.topics.filter_move_result_pub, 20);
    m_statePub = nh()->advertise<bigbang_eurobot::MonteCarloState>(params.topics.state_pub, 20);
    m_returnToStart = nh()->advertiseService(params.topics.return_to_start, &MonteCarlo::returnToStart, this);
    m_discardBelief.shutdown();
    m_discardBelief = nh()->advertiseService(params.topics.discard_service, &MonteCarlo::discardBelief, this);
    m_tfMsg.header.frame_id = params.topics.frame_id;
    m_tfMsg.child_frame_id = params.topics.child_frame_id;
    m_updateTimer->start(params.time_step * 1000);
    m_moveResultMsg.source_id = params.outputs.move_source_id;
    m_pointCloud.header.frame_id = params.topics.base_frame_id;
    m_particles.setPosition({params.start_x, params.start_y, params.start_theta});
    ROS_INFO_STREAM("RESAMPLE ON MEASURE: " << params.resample_on_measure);
}

bool MonteCarlo::returnToStart(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp)
{
    m_particles.setPosition({params.start_x, params.start_y, params.start_theta});
    return true;
}

bool MonteCarlo::discardBelief(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp)
{
    m_particles.discardBelief();
    return true;
}

void MonteCarlo::applyMovesStep()
{
    if (m_badStateFor > params.discard_after_bad_for) {
        ROS_WARN("Lost for too long! Discarding position!");
        m_particles.discardBelief();
        m_badStateFor = 0;
        return;
    }
    bool moveOk = false;
    MeasurePair move = collectMoves(&moveOk);
    auto lastMedian = m_particles.currentMedian();
    if (moveOk) {
        m_particles.applyMove(move.vec);
    }
    auto diff = PositionF(m_particles.currentMedian() - lastMedian);
    m_moveResultMsg.dx = diff.getX();
    m_moveResultMsg.dy = diff.getY();
    m_moveResultMsg.dtheta = diff.getTheta();
    auto dispersion = std::min(1 / m_particles.density(), params.max_uncertainty);
    m_moveResultMsg.covariance.fill(dispersion);
    m_positionMsg.covariance.fill(dispersion);
    m_positionMsg.x = m_particles.currentMedian().getX();
    m_positionMsg.y = m_particles.currentMedian().getY();
    m_positionMsg.theta = m_particles.currentMedian().getTheta();
    publishState();
}

void MonteCarlo::publishState()
{
    ROS_INFO_STREAM_THROTTLE(2, "[PARTICLES]: Density: " << m_particles.density() * 100 << "%");
    m_state.density = m_particles.density();
    m_state.is_ok = m_state.density >= params.density.ok_level;
    m_state.is_mid = m_state.density < params.density.ok_level && m_state.density >= params.density.mid_level;
    m_state.is_bad = m_state.density < params.density.mid_level;
    if (!m_state.is_ok) {
        m_badStateFor += m_updateTimer->interval() / 1000.;
    } else {
        m_badStateFor = 0;
    }
    m_statePub.publish(m_state);
    m_moveResultPub.publish(m_moveResultMsg);
    m_resultPub.publish(m_positionMsg);
    m_tfMsg.header.seq++;
    m_tfMsg.header.stamp = ros::Time::now();
    m_tfMsg.transform.rotation = msgFromTheta(m_positionMsg.theta);
    m_tfMsg.transform.translation.x = m_positionMsg.x;
    m_tfMsg.transform.translation.y = m_positionMsg.y;
    m_tfBroad.sendTransform(m_tfMsg);
    if (params.send_particles) {
        m_particles.fillPointCloud(m_pointCloud);
        m_pointCloud.header.seq++;
        m_pointCloud.header.stamp = ros::Time::now();
        m_pointCloudPub.publish(m_pointCloud);
    }
}

void MonteCarlo::moveCb(const bigbang_eurobot::Move2dConstPtr &msg)
{
    auto cov = CovT();
    for (int i = 0; i < msg->covariance.static_size; ++i) {
        cov(i/3, i%3) = msg->covariance[i];
    }
    checkCovarianceDiagMin(cov, params.min_uncertainty);
    auto it = m_currentMoves.find(msg->source_id);
    if (it == m_currentMoves.end()) {
        m_currentMoves.try_emplace(msg->source_id, MeasurePair{VecT(msg->dx, msg->dy, msg->dtheta), cov});
    } else {
        it->second.vec += VecT(msg->dx, msg->dy, msg->dtheta);
        it->second.cov += cov;
        it->second.vec *= 0.5;
        it->second.cov *= 0.5;
    }
}

void MonteCarlo::beaconsCb(const bigbang_eurobot::LaserBeaconsConstPtr &msg)
{
    if (params.resample_on_measure) {
        m_particles.applyMeasure(msg);
    }
}

void MonteCarlo::rvizInitialCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    m_particles.setPosition(PositionF(msg->pose.pose.position.x, msg->pose.pose.position.y, thetaFromMsg(msg->pose.pose.orientation)));
    ROS_WARN("Received new pos estimate: x:%f; y:%f; th:%f",
             m_particles.currentMedian().getX(),
             m_particles.currentMedian().getY(),
             m_particles.currentMedian().getTheta());
}

MeasurePair MonteCarlo::collectMoves(bool *ok)
{
    MeasurePair result{{}, CovT(MAX_UNC_9)};
    if (m_currentMoves.empty()) {
        if (ok) *ok = false;
        return result;
    }
    for (auto& [_, move]: m_currentMoves) {
        auto currentCov = move.cov;
        checkCovarianceDiagMin(currentCov, params.min_uncertainty);
        auto currentMove = move.vec;
        mergeMeasures(result, {currentMove, currentCov}, params.decomposition_alg);
    }
    if (ok) *ok = true;
    m_currentMoves.clear();
    return result;
}

NODE_MAIN(MonteCarlo, "monte_carlo")
