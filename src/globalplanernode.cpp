#include "global_planer/globalplanernode.hpp"

using namespace bigbang_eurobot;
using namespace cv;
GlobalPlaner::GlobalPlaner(const NodeSettings &settings) :
    NodeBase(settings),
    m_params(),
    m_graph(m_params._nodes_batch_size),
    m_positionSub(),
    m_costmapSub(),
    m_rvizTargetSub(),
    m_targetSub(),
    m_pathPub(),
    m_tfMsg(),
    m_pathMsg(),
    m_tfBroad(),
    m_position(),
    m_updatePathTimer(new QTimer(this)),
    m_target(),
    m_targetStatus()
{
    updateParamsImpl();
    connect(this, &GlobalPlaner::planningFailed, &GlobalPlaner::onPlanningFailed);
}


void GlobalPlaner::updateParamsImpl(const QString &name)
{
    m_params.update(name);
    m_updatePathTimer->start(m_params.update_time * 1000);
    m_updatePathTimer->callOnTimeout(this, &GlobalPlaner::update);
    m_positionSub = nh()->subscribe(m_params.topics.position_sub.toStdString(), 20, &GlobalPlaner::positionCb, this);
    m_cancelSub = nh()->subscribe(m_params.topics.cancel_sub.toStdString(), 20, &GlobalPlaner::cancelTarget, this);
    m_costmapSub = nh()->subscribe(m_params.topics.costmap_sub.toStdString(), 20, &GlobalPlaner::costmapCb, this);
    m_rvizTargetSub = nh()->subscribe(m_params.topics.rviz_target_sub.toStdString(), 20, &GlobalPlaner::rvizTargetCb, this);
    m_targetSub = nh()->subscribe(m_params.topics.target_sub.toStdString(), 20, &GlobalPlaner::targetCb, this);
    m_statusSub = nh()->subscribe(m_params.topics.local_planer_status_sub.toStdString(), 20, &GlobalPlaner::statusCb, this);
    m_pathPub = nh()->advertise<nav_msgs::Path>(m_params.topics.path_out_pub.toStdString(), 20);
    m_tfMsg.child_frame_id = m_params.topics.child_frame_id.toStdString();
    m_tfMsg.header.frame_id = m_params.topics.frame_id.toStdString();
    m_tfMsg.transform.rotation = msgFromTheta(0);
    m_pathMsg.header.frame_id = m_params.topics.child_frame_id.toStdString().c_str();
    m_pathMsg.poses.reserve(m_params.reserve_in_path_size);
    m_graph.updateBatchSize(m_params.nodes_batch_size);
    m_open.reserve(m_params.nodes_batch_size);
    m_covered.reserve(m_params.nodes_batch_size);
}

void GlobalPlaner::statusCb(const PlanerStatusConstPtr &msg)
{
    if (!m_targetStatus.canceled &&
        m_timeSinceNewTarget > m_params.min_time_for_target &&
        msg->idle_for >= m_params.consider_reached_after)
    {
        ROS_WARN("Global planer finished work!");
        cancelTarget();
    }
}

void GlobalPlaner::updateParams(const QString &name)
{
    updateParamsImpl(name);
}

const QString &GlobalPlaner::baseFrameId() const
{
    return m_params.topics.frame_id;
}

void GlobalPlaner::onPlanningFailed()
{
    ROS_WARN_THROTTLE(0.4, "Planning failed!");
    pubEmptyPath();
}

void GlobalPlaner::onPlanningSuccess()
{
    ROS_DEBUG_THROTTLE(0.4, "Planning OK");
}

void GlobalPlaner::newTarget(float x, float y, float theta)
{
    m_timeSinceNewTarget = 0;
    ROS_WARN_STREAM("[Global Planer] New target: " << x << ',' << y << ',' << theta);
    m_target.coord = m_costmap.metersPosToCells(x, y); 
    m_target.theta = theta;
    m_targetStatus.reset();
}

void GlobalPlaner::cancelTarget(const std_msgs::EmptyConstPtr &msg)
{
    ROS_WARN_THROTTLE(1, "[Global Planer] Target cancelled!");
    if (!m_targetStatus.reached) {
        m_targetStatus.canceled = true;
        m_targetStatus.reached = true;
        pubEmptyPath();
    }
}

void GlobalPlaner::positionCb(const Measure2dConstPtr &msg)
{
    m_position = *msg;
}

void GlobalPlaner::costmapCb(const nav_msgs::OccupancyGridConstPtr &msg)
{
    m_costmap.setMeta(CostmapMetadata(msg->info.width, msg->info.height, msg->info.resolution));
    m_costmap.fillFromVector(msg->data);
}

void GlobalPlaner::pubEmptyPath()
{
    m_pathMsg.poses.clear();
    publishPath();
}
quint32 GlobalPlaner::getBestOpenIndex()
{
    quint32 bestNode = 0;
    float minCost = std::numeric_limits<float>::max();
    for (auto open : m_open) {
        auto &node = graphAt(open);
        if (node.totalCost < minCost) {
            minCost = node.totalCost;
            bestNode = open;
        }
    }
    return bestNode;
}

void GlobalPlaner::update()
{
    m_timeSinceNewTarget+=m_params.update_time;
    if (m_targetStatus.canceled) return;
    m_graph.reset();
    m_covered.clear();
    m_open.clear();
    m_covered.clear();
    m_currentCount = 0;
    append(PlanerNode(m_costmap.metersPosToCells(m_position.x, m_position.y), 0, 0, m_position.theta));
    spawnChildren(0);
    while (!m_targetStatus.reached && !m_targetStatus.canceled) {
        auto minCost = getBestOpenIndex();
        if (!minCost || m_currentCount++>=m_params.max_points) {
            m_targetStatus.canceled = true;
            break;   
        }
        spawnChildren(minCost);
    }
    if (!m_targetStatus.canceled) {
        emit planningSuccess();
        walkGraphBackwards();
    } else {
        emit planningFailed();
        pubEmptyPath();
    }
    m_targetStatus.reset();
}

float GlobalPlaner::getCost(const Coord &coord, bool isDiagonalStep) const
{
    auto dist = norm(m_target.coord - coord);
    auto stepCost = isDiagonalStep 
                ? aStar().cell_cost * aStar().diagonal_coeff
                : aStar().cell_cost; 
    auto costMapAfterCoeff = m_costmap.at(coord) * aStar().costmap_to_node_cost_coeff;
    return dist * aStar().cell_cost + stepCost + costMapAfterCoeff;
}

void GlobalPlaner::spawnChildren(quint32 parentIndex)
{
    auto x = graphAt(parentIndex).coord.getX();
    auto y = graphAt(parentIndex).coord.getY();
    m_open.erase(parentIndex);
    for (int dx = -1; dx < 2; ++dx) {
        for (int dy = -1; dy < 2; ++dy) {
            if (!dx && !dy) continue;
            auto currentCoord = Coord{x+dx, y+dy};
            if (currentCoord == m_target.coord) {
                m_targetStatus.reached = true;
                m_open.insert(append(PlanerNode(m_target.coord, parentIndex, 0, m_target.theta)));
                graphAt(0).theta = m_position.theta;
                return;
            }
            if (m_covered.find(currentCoord) != m_covered.end()) continue;
            if (!m_costmap.coordIsValid(currentCoord)) continue; 
            if (m_costmap.at(currentCoord) > aStar().max_cost) continue;
            auto currentCost = getCost(currentCoord, dx && dy);
            m_covered.emplace(currentCoord);
            m_open.insert(append(PlanerNode(currentCoord, parentIndex, currentCost)));
        }   
    }
}

void GlobalPlaner::publishPath()
{
    ++m_tfMsg.header.seq;
    m_tfMsg.header.stamp = ros::Time::now();
    m_tfBroad.sendTransform(m_tfMsg);
    ROS_DEBUG_STREAM("Sending new path with: " << m_pathMsg.poses.size() << " points");
    ++m_pathMsg.header.seq;
    m_pathPub.publish(m_pathMsg);
}

void GlobalPlaner::walkGraphBackwards()
{
    auto frameId = m_params.topics.child_frame_id.toStdString();
    auto *node = &lastNode();
    auto endTheta = node->theta;
    auto startTheta = graphAt(0).theta;
    m_pathMsg.poses.clear();
    m_pathMsg.poses.emplace_back(node->toPose(m_costmap, frameId));
    auto count = 0;
    while (!isRoot(*node)) {
        node = &graphAt(node->parent);
        count++;
    }
    auto diff = std::remainder(endTheta - startTheta, 2 * M_PI);
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    auto thetaStep = diff / count;
    count = 0;
    node = &lastNode();
    while (!isRoot(*node)) {
        node = &graphAt(node->parent);
        node->theta = endTheta - thetaStep * ++count;
        m_pathMsg.poses.emplace_back(node->toPose(m_costmap, frameId));
    }
    std::reverse(m_pathMsg.poses.begin(), m_pathMsg.poses.end());
    auto start_theta = m_pathMsg.poses;
    publishPath();
}

void GlobalPlaner::rvizTargetCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (m_params.enable_rviz_target) {
        newTarget(msg->pose.position.x, msg->pose.position.y, thetaFromMsg(msg->pose.orientation));
    }
}

void GlobalPlaner::targetCb(const geometry_msgs::PointConstPtr &msg)
{
    newTarget(msg->x, msg->y, msg->z);
}

NODE_MAIN(GlobalPlaner, "global_planer")
