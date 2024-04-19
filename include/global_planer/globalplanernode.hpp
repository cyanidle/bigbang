#ifndef GLOBAL_PLANER_NODE
#define GLOBAL_PLANER_NODE
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "bigbang/types/costmap.hpp"
#include "bigbang/types/rosparams.h"
#include "bigbang/types/cachedgraph.hpp"
#include <unordered_set>
#include "bigbang_eurobot/Measure2d.h"
#include "bigbang_eurobot/PlanerStatus.h"
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "nodebase.h"
#include <std_msgs/Empty.h>
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

struct PlanerNode
{
    Coord coord;
    float theta;
    quint32 parent;
    float totalCost{0};
    PlanerNode() : coord(), theta(), parent(), totalCost() {}
    PlanerNode(const Coord &coord, quint32 parent, float totalCost = 0, float theta = 0) :
        coord(coord), theta(theta), parent(parent), totalCost(totalCost) {}
    template<typename T, T max>
    geometry_msgs::PoseStamped toPose(const Costmap<T, max> &costmap, const std::string &childFrame) const {
        auto msg = geometry_msgs::PoseStamped();
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        msg.header.frame_id = childFrame;
        msg.header.stamp = ros::Time::now();
        auto realCoord = costmap.metersPosFromCells(coord.getX(), coord.getY());
        msg.pose.position.x = realCoord.getX();
        msg.pose.position.y = realCoord.getY();
        tf::quaternionTFToMsg(q, msg.pose.orientation);
        return msg;
    }
};
Q_DECLARE_TYPEINFO(PlanerNode, Q_MOVABLE_TYPE);

struct AStarParams : public RosParams
{
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(double, costmap_to_node_cost_coeff, 5)
    SERIAL_FIELD(double, cell_cost, 10)
    SERIAL_FIELD(double, diagonal_coeff, 1.25)
    SERIAL_FIELD(int, max_cost, 35)
};

struct GlobalPlanerTopics : public RosParams
{
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(QString, position_sub, "filtered_pos")
    SERIAL_FIELD(QString, costmap_sub, "costmap")
    SERIAL_FIELD(QString, rviz_target_sub, "move_base_simple/goal")
    SERIAL_FIELD(QString, target_sub, "global_planer/target")
    SERIAL_FIELD(QString, cancel_sub, "global_planer/cancel")
    SERIAL_FIELD(QString, local_planer_status_sub, "planer_status")
    SERIAL_FIELD(QString, path_out_pub, "global_plan")
    SERIAL_FIELD(QString, frame_id, "costmap")
    SERIAL_FIELD(QString, child_frame_id, "global_plan")
};

struct GlobalPlanerParams : public RosParams
{
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_NEST(AStarParams, a_star_params, DEFAULT)
    SERIAL_FIELD(int, nodes_batch_size, 10000)
    SERIAL_FIELD(int, reserve_in_path_size, 60)
    SERIAL_FIELD(bool, enable_rviz_target, true)
    SERIAL_FIELD(double, update_time, 0.05)
    SERIAL_FIELD(int, max_points, 2000)
    SERIAL_FIELD(double, consider_reached_after, 1.5)
    SERIAL_FIELD(double, delay_between_fails, 0.05)
    SERIAL_FIELD(int, max_fails_before_abort, 3)
    SERIAL_FIELD(int, min_time_for_target, 0.5)
    SERIAL_NEST(GlobalPlanerTopics, topics, DEFAULT)
    SERIAL_POST_INIT(postInit);
    
    quint32 _nodes_batch_size;
    void postInit() {
        _nodes_batch_size = static_cast<quint32>(nodes_batch_size);
    }
};

struct TargetStatus
{
    bool canceled{true};
    bool reached{false};
    void reset() {
        canceled = false;
        reached = false;
    }
};

class GlobalPlaner : public NodeBase
{
    Q_OBJECT
public: 
    GlobalPlaner(const NodeSettings &settings);
    const GlobalPlanerParams &params() const {return m_params;}
    void updateParams(const QString &name = "") override;
    const QString &baseFrameId() const override;
signals:
    void planningFailed();
    void planningSuccess();
    void targetAborted();

public slots:
    void newTarget(float x, float y, float theta);
    void cancelTarget(const std_msgs::EmptyConstPtr &msg = {});
private slots:
    void onPlanningFailed();
    void onPlanningSuccess();

private:
    void pubEmptyPath();
    quint32 getBestOpenIndex();
    void spawnChildren(quint32 parentIndex);
    void update();
    float getCost(const Coord &coord, bool isDiagonalStep) const;
    void publishPath();
    void walkGraphBackwards();
    void rvizTargetCb(const geometry_msgs::PoseStampedConstPtr &msg);
    void targetCb(const geometry_msgs::PointConstPtr &msg);
    void positionCb(const bigbang_eurobot::Measure2dConstPtr &msg);
    void statusCb(const bigbang_eurobot::PlanerStatusConstPtr &msg);
    void costmapCb(const nav_msgs::OccupancyGridConstPtr &msg);
    inline PlanerNode& graphAt(int index) {
        return m_graph.at(index);
    }
    inline PlanerNode& lastNode() {
        return m_graph.at(m_graph.size()-1);
    }
    inline bool isRoot(const PlanerNode& node) {
        return !node.parent;
    }
    template <typename U>
    inline quint32 append(U&& node) {
        return m_graph.append(std::forward<U>(node));
    }
    inline const AStarParams &aStar() const {return m_params.a_star_params;}
    void updateParamsImpl(const QString &name = {});

    GlobalPlanerParams m_params;
    CachedGraph<PlanerNode> m_graph;
    ros::Subscriber m_positionSub;
    ros::Subscriber m_costmapSub;
    ros::Subscriber m_rvizTargetSub;
    ros::Subscriber m_targetSub;
    ros::Subscriber m_cancelSub;
    ros::Subscriber m_statusSub;
    ros::Publisher m_pathPub;
    geometry_msgs::TransformStamped m_tfMsg;
    nav_msgs::Path m_pathMsg;
    tf::TransformBroadcaster m_tfBroad;
    bigbang_eurobot::Measure2d m_position;

    QTimer *m_updatePathTimer;
    PlanerNode m_target;
    TargetStatus m_targetStatus;
    Costmap<int8_t, 100> m_costmap;
    int m_currentCount{0};
    float m_timeSinceNewTarget{0};
    std::unordered_set<quint32> m_open{};
    std::unordered_set<Coord> m_covered{};
};

#endif
