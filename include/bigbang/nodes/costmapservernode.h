#ifndef COSTMAP_SERVER_H
#define COSTMAP_SERVER_H

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <QtCore/QtCore>
#include "nodebase.h"
#include "bigbang/types/costmap.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <bigbang_eurobot/MapObject.h>

Q_DECLARE_TYPEINFO(bigbang_eurobot::MapObject, Q_MOVABLE_TYPE);

struct CostmapTopics : public RosParams
{
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(QString, costmap_pub, "costmap")
    SERIAL_FIELD(QString, objects_sub, "obstacles")
    SERIAL_FIELD(QString, rviz_points, "/clicked_point")
    SERIAL_FIELD(QString, frame_id, "map")
    SERIAL_FIELD(QString, child_frame_id, "costmap")
};

struct CostmapParams : public RosParams
{
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(int, width, 101)
    SERIAL_FIELD(int, height, 151)
    SERIAL_FIELD(float, resolution, 0.02f) // m per side of pixel
};

struct CostmapServerParams : public RosParams
{
    Q_GADGET
    IS_SERIALIZABLE  
    SERIAL_FIELD(int, update_rate_ms, 80)
    SERIAL_FIELD(int, keep_rviz_points_ms, 15000)
    SERIAL_FIELD(bool, debug, true)
    SERIAL_FIELD(bool, ignore_all_outside, true)
    SERIAL_FIELD(QString, image_path, "~/catkin_ws/src/bigbang/config/costmap.png")
    SERIAL_NEST(CostmapParams, costmap, DEFAULT)
    SERIAL_NEST(CostmapTopics, topics, DEFAULT)
    SERIAL_NEST(InflateSettings, inflate, DEFAULT)
    SERIAL_NEST(InflateSettings, inflate_static, DEFAULT)
};

class CostmapServer : public NodeBase
{
    Q_OBJECT
public:
    CostmapServer(const NodeSettings &settings);
    const CostmapServerParams &params() const {return m_params;}
    const QString &baseFrameId() const override;
    ~CostmapServer();
public slots:
    void updateParams(const QString &name = "") override final;
private slots:
    void rvizPointCb(const geometry_msgs::PointStampedConstPtr &msg);
    void objectsCb(const bigbang_eurobot::MapObjectConstPtr &msg);

    void updateMap();
    void sendMap();
private:
    void updateObjects();
    void initMap();

    nav_msgs::OccupancyGrid m_gridMsg;
    CostmapServerParams m_params;
    QTimer* m_updateTimer;
    Costmap<int8_t, 100> m_costmap;
    Costmap<int8_t, 100> m_currentReceived;
    Costmap<int8_t, 100> m_inflatedCostmap;
    Costmap<int8_t, 100> m_receivedRvizPoints;
    Costmap<int8_t, 100> m_receivedLaserPoints;
    ros::Publisher m_mapPublisher;
    ros::Subscriber m_rvizPointsSub;
    ros::Subscriber m_objectsSub;
    tf::TransformBroadcaster m_tf;
    geometry_msgs::TransformStamped m_tfMsg;
    QTimer* m_resetRvizPoints;
    boost::unordered_map<float, std::vector<DPoint>> m_dpointsForSizes;
    boost::unordered_map<int, boost::unordered_map<int, bigbang_eurobot::MapObject>> m_objects;
    std::vector<int> idsToRemove{};
};
#endif // COSTMAP_SERVER_H
