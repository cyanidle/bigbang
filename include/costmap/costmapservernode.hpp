#ifndef COSTMAP_SERVER_H
#define COSTMAP_SERVER_H

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <QtCore/QtCore>
#include "nodebase.h"
#include <string>
#include "bigbang/types/costmap.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <bigbang_eurobot/MapObject.h>
#include "describe/describe.hpp"

struct CostmapTopics {
    std::string costmap_pub = "costmap";
    std::string objects_sub = "obstacles";
    std::string rviz_points = "/clicked_point";
    std::string frame_id = "map";
    std::string child_frame_id = "costmap";
};
DESCRIBE(CostmapTopics,
         &_::costmap_pub, &_::objects_sub, &_::rviz_points,
         &_::frame_id, &_::child_frame_id)

struct CostmapParams {
    int width = 101;
    int height = 151;
    float resolution = 0.02f; // m per side of pixel
};
DESCRIBE(CostmapParams, &_::width, &_::height, &_::resolution)

struct CostmapServerParams
{
    int update_rate_ms = 80;
    int keep_rviz_points_ms = 15000;
    bool debug = true;
    bool ignore_all_outside = true;
    std::string image_path = "~/bigbang/config/costmap.png";
    CostmapParams costmap;
    CostmapTopics topics;
    InflateSettings inflate;
    InflateSettings inflate_static;
};
DESCRIBE(CostmapServerParams,
         &_::update_rate_ms, &_::keep_rviz_points_ms, &_::debug,
         &_::ignore_all_outside, &_::image_path, &_::costmap,
         &_::topics, &_::inflate, &_::inflate_static)

class CostmapServer
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
