#include "costmap/costmapservernode.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <tf/LinearMath/Quaternion.h>

using namespace cv;

void CostmapServer::updateParams(const QString &name) {
    m_params.update(name);
    m_updateTimer->setInterval(m_params.update_rate_ms);
    m_objectsSub = nh()->subscribe(params().topics.objects_sub.toStdString(), 800, &CostmapServer::objectsCb, this);
    m_rvizPointsSub = nh()->subscribe(params().topics.rviz_points.toStdString(), 20, &CostmapServer::rvizPointCb, this);
    m_mapPublisher = nh()->advertise<nav_msgs::OccupancyGrid>(params().topics.costmap_pub.toStdString(), 20);
    initMap();
    if (!m_resetRvizPoints) {
        m_resetRvizPoints = new QTimer(this);
        m_resetRvizPoints->callOnTimeout([this](){
            m_receivedRvizPoints.resetValues();
        });
        m_resetRvizPoints->setSingleShot(true);
    }
    m_resetRvizPoints->setInterval(params().keep_rviz_points_ms);
}


void CostmapServer::objectsCb(const bigbang_eurobot::MapObjectConstPtr &msg)
{
    if (m_params.ignore_all_outside &&
        !(0 <= msg->x && msg->x <= m_costmap.meta().width) &&
        !(0 <= msg->y && msg->y <= m_costmap.meta().height))
    {
        return;
    }
    m_objects[msg->source_id][msg->id] = *msg;
    if (m_dpointsForSizes.find(msg->size) == m_dpointsForSizes.end()) {
        m_dpointsForSizes[msg->size] = m_receivedLaserPoints.createDPoints(msg->size/2);
    }
}

void CostmapServer::rvizPointCb(const geometry_msgs::PointStampedConstPtr &msg)
{
    auto coord = m_receivedRvizPoints.metersPosToCells(msg->point.x, msg->point.y);
    m_receivedRvizPoints.set(coord, 100);
    m_resetRvizPoints->setInterval(params().keep_rviz_points_ms);
    m_resetRvizPoints->start();
}

CostmapServer::CostmapServer(const NodeSettings &settings):
    NodeBase(settings),
    m_gridMsg(),
    m_params(),
    m_updateTimer(new QTimer(this)),
    m_costmap(),
    m_inflatedCostmap(),
    m_receivedRvizPoints(),
    m_receivedLaserPoints(),
    m_mapPublisher(),
    m_rvizPointsSub(),
    m_tf(),
    m_tfMsg(),
    m_resetRvizPoints()
{ 
    updateParams();
    idsToRemove.reserve(100);
    m_tfMsg.child_frame_id = params().topics.child_frame_id.toStdString();
    m_tfMsg.header.frame_id = params().topics.frame_id.toStdString();
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q, m_tfMsg.transform.rotation);
    m_updateTimer->callOnTimeout(this, &CostmapServer::updateMap);
    ROS_INFO_STREAM("Costmap server started!");
}

const QString &CostmapServer::baseFrameId() const
{
    return m_params.topics.child_frame_id;
}

CostmapServer::~CostmapServer()
{
    qDebug("Goodbye");
    ROS_WARN("Goodbye");
}

void CostmapServer::initMap() 
{
    Mat rimage = imread(params().image_path.toStdString().c_str(), IMREAD_GRAYSCALE);
    Mat image;
    flip(rimage, image, RotateFlags::ROTATE_180);
    if (!image.size) {
        throw std::runtime_error("Empty source image for costmap!");
    }
    ROS_DEBUG_STREAM("Got image: \n" << image);
    if (image.cols != params().costmap.width || image.rows != params().costmap.height) {
        throw std::runtime_error("Costmap params are not coherent with image loaded!");
    }
    auto flat = image.reshape(1, image.total());
    flat *= 100.0f/255.0f;
    std::vector<uchar> vec = flat.isContinuous()
                                                ? flat 
                                                : flat.clone();
    std::vector<uchar> reversed(vec.rbegin(), vec.rend()); 
    m_costmap.setMeta({
        params().costmap.width,
        params().costmap.height,
        params().costmap.resolution
    });
    m_dpointsForSizes.reserve(50);
    m_dpointsForSizes.clear();
    m_costmap.fillFromVector(copyVectorAs<uint8_t>(reversed));
    m_costmap.updateSettings(params().inflate_static);

    m_receivedRvizPoints.copyFrom(m_costmap);
    m_receivedRvizPoints.updateSettings(params().inflate);
    m_receivedLaserPoints.copyFrom(m_receivedRvizPoints);
    m_currentReceived.copyFrom(m_receivedRvizPoints);
    m_currentReceived.resetValues();
    m_receivedRvizPoints.resetValues();
    m_receivedLaserPoints.resetValues();
    m_gridMsg.info.resolution = params().costmap.resolution;
    m_gridMsg.info.width = params().costmap.width;
    m_gridMsg.info.height = params().costmap.height;
    m_costmap = m_costmap.inflated();
    ROS_INFO_STREAM("Costmap read successfully!");
    m_updateTimer->start();
}
void CostmapServer::updateMap()
{
    auto start = std::chrono::high_resolution_clock::now();
    m_inflatedCostmap = m_costmap;
    updateObjects();
    m_currentReceived.addCosts(m_receivedRvizPoints).addCosts(m_receivedLaserPoints);
    m_currentReceived.inflateInto(m_inflatedCostmap);
    m_currentReceived.resetValues();
    sendMap();
    if (!params().debug) {
        return;
    }
    auto passed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
    ROS_DEBUG_STREAM("Time for update (ms): " << passed / 1000.f);
}

void CostmapServer::updateObjects()
{
    m_receivedLaserPoints.resetValues();
    for (auto &source : m_objects) {
        for (auto &obj : source.second){
            obj.second.ttl -= m_params.update_rate_ms;
            if (obj.second.ttl <= 0) {
                idsToRemove.push_back(obj.first);
                continue;
            }
            auto realCoord = m_receivedLaserPoints.metersPosToCells(obj.second.x, obj.second.y);
            m_receivedLaserPoints.setAll(realCoord, m_dpointsForSizes[obj.second.size], 100);
        }
        for (const auto &id: idsToRemove) {
            source.second.erase(id);
        }
        idsToRemove.clear();
    }
}

void CostmapServer::sendMap()
{
    auto stamp = ros::Time::now();
    if (m_inflatedCostmap.isEmpty()) {
        return;
    }
    m_gridMsg.header.frame_id = params().topics.frame_id.toStdString();
    m_gridMsg.header.stamp = stamp;
    m_gridMsg.data = std::move(m_inflatedCostmap.data());
    m_mapPublisher.publish(m_gridMsg);   
    ++m_tfMsg.header.seq;
    m_tfMsg.header.stamp = stamp;
    m_tf.sendTransform(m_tfMsg);
}

NODE_MAIN(CostmapServer, "costmap_server")
