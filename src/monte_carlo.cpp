#include "monte_carlo/monte_carlo.hpp"
#define MAX_UNC m_params.max_uncertainty
#define MAX_UNC_9   MAX_UNC, 0, 0, \
                    0, MAX_UNC, 0, \
                    0, 0, MAX_UNC

MonteCarlo::MonteCarlo(const NodeSettings &settings) :
    NodeBase(settings),
    m_params(),
    m_particles(m_params.particles),
    m_updateTimer(new QTimer(this))
{
    updateParams();
    m_updateTimer->callOnTimeout(this, &MonteCarlo::applyMovesStep);
}

void MonteCarlo::updateParams(const QString &name)
{
    m_params.update(name);
    m_particles = Particles{m_params.particles};
    m_moveSub = nh()->subscribe(m_params.topics.move_sub.toStdString(), 20, &MonteCarlo::moveCb, this);
    m_measureSub = nh()->subscribe(m_params.topics.beacons_sub.toStdString(), 20, &MonteCarlo::beaconsCb, this);
    m_rvizInitialSub = nh()->subscribe(m_params.topics.rviz_set_point_sub.toStdString(), 20, &MonteCarlo::rvizInitialCb, this);
    m_resultPub = nh()->advertise<bigbang_eurobot::Measure2d>(m_params.topics.filter_result_pub.toStdString(), 20);
    m_pointCloudPub = nh()->advertise<sensor_msgs::PointCloud>(m_params.topics.point_cloud_pub.toStdString(), 20);
    m_moveResultPub = nh()->advertise<bigbang_eurobot::Move2d>(m_params.topics.filter_move_result_pub.toStdString(), 20);
    m_statePub = nh()->advertise<bigbang_eurobot::MonteCarloState>(m_params.topics.state_pub.toStdString(), 20);
    m_returnToStart = nh()->advertiseService(m_params.topics.return_to_start.toStdString(), &MonteCarlo::returnToStart, this);
    m_discardBelief.shutdown();
    m_discardBelief = nh()->advertiseService(m_params.topics.discard_service.toStdString(), &MonteCarlo::discardBelief, this);
    m_tfMsg.header.frame_id = m_params.topics.frame_id.toStdString();
    m_tfMsg.child_frame_id = m_params.topics.child_frame_id.toStdString();
    m_updateTimer->start(m_params.time_step * 1000);
    m_moveResultMsg.source_id = m_params.outputs.move_source_id;
    m_pointCloud.header.frame_id = m_params.topics.base_frame_id.toStdString();
    m_particles.setPosition({m_params.start_x, m_params.start_y, m_params.start_theta});
    ROS_INFO_STREAM("RESAMPLE ON MEASURE: " << m_params.resample_on_measure);
}

bool MonteCarlo::returnToStart(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp)
{
    m_particles.setPosition({m_params.start_x, m_params.start_y, m_params.start_theta});
    return true;
}

bool MonteCarlo::discardBelief(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp)
{
    m_particles.discardBelief();
    return true;
}

const QString &MonteCarlo::baseFrameId() const
{
    return m_params.topics.base_frame_id;
}

void MonteCarlo::applyMovesStep()
{
    if (m_badStateFor > m_params.discard_after_bad_for) {
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
    auto dispersion = std::min(1 / m_particles.density(), m_params.max_uncertainty);
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
    m_state.is_ok = m_state.density >= m_params.density.ok_level;
    m_state.is_mid = m_state.density < m_params.density.ok_level && m_state.density >= m_params.density.mid_level;
    m_state.is_bad = m_state.density < m_params.density.mid_level;
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
    if (m_params.send_particles) {
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
    checkCovarianceDiagMin(cov, m_params.min_uncertainty);
    if (!m_currentMoves.contains(msg->source_id)) {
        m_currentMoves.insert(msg->source_id, {VecT(msg->dx, msg->dy, msg->dtheta), cov});
    } else {
        m_currentMoves[msg->source_id].vec += VecT(msg->dx, msg->dy, msg->dtheta);
        m_currentMoves[msg->source_id].cov += cov;
        m_currentMoves[msg->source_id].vec *= 0.5;
        m_currentMoves[msg->source_id].cov *= 0.5;
    }
}

void MonteCarlo::beaconsCb(const bigbang_eurobot::LaserBeaconsConstPtr &msg)
{
    if (m_params.resample_on_measure) {
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
    if (m_currentMoves.isEmpty()) {
        if (ok) *ok = false;
        return result;
    }
    for (auto key = m_currentMoves.keyBegin(); key != m_currentMoves.keyEnd(); ++key) {
        auto currentCov = m_currentMoves[*key].cov;
        checkCovarianceDiagMin(currentCov, m_params.min_uncertainty);
        auto currentMove = m_currentMoves[*key].vec;
        mergeMeasures(result, {currentMove, currentCov}, m_params.decomposition_alg_);
    }
    if (ok) *ok = true;
    m_currentMoves.clear();
    return result;
}

void MonteCarloParams::prvInit() {
    ROS_INFO_STREAM("Using Decomposition algorithm: " << decomposition_alg.toStdString());
    ROS_INFO_STREAM("Available Decomposition algorithms: 'cholesky', 'eigen', 'svd', 'lu', 'normal', 'qr'");
    if (decomposition_alg.toLower() == "cholesky") {
        decomposition_alg_ = cv::DECOMP_CHOLESKY;
    } else if (decomposition_alg.toLower() == "eigen") {
        decomposition_alg_ = cv::DECOMP_EIG;
    } else if (decomposition_alg.toLower() == "svd") {
        decomposition_alg_ = cv::DECOMP_SVD;
    } else if (decomposition_alg.toLower() == "lu") {
        decomposition_alg_ = cv::DECOMP_LU;
    } else if (decomposition_alg.toLower() == "normal") {
        decomposition_alg_ = cv::DECOMP_NORMAL;
    } else if (decomposition_alg.toLower() == "qr") {
        decomposition_alg_ = cv::DECOMP_QR;
    } else {
        ROS_WARN_STREAM("Unknown matrix decomposition algorithm: " + decomposition_alg.toStdString());
    }
}

NODE_MAIN(MonteCarlo, "monte_carlo")
