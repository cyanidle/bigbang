#include "nodebase.h"
#include "bigbang/types/costmap.hpp"
#include <visualization_msgs/Marker.h>
#include <QThread>

using namespace visualization_msgs;

NodeBase::NodeBase(const NodeSettings &settings) :
    QObject(settings.app),
    m_settings(settings),
    m_nh(),
    m_watchdog(new QTimer(this))
{
    m_watchdog->setInterval(settings.spin_rate_ms);
    m_watchdog->callOnTimeout(this, &NodeBase::watchdog);
    m_watchdog->start();
    thread()->setPriority(QThread::HighPriority);
    m_reconfSub = m_nh.subscribe<bigbang_eurobot::Reconfigure>("/reconfigure", 20, &NodeBase::reconfCb, this);
    m_markerPub = m_nh.advertise<Marker>("/markers", 20);
    ROS_WARN_STREAM("Node (" << ros::this_node::getName() << ") is listening to '/reconfigure' topic for own name!");
}

NodeBase::~NodeBase()
{
    ros::shutdown();
}

void NodeBase::watchdog() {
    if (!ros::ok()) {
        shutdown();
    }
    ros::spinOnce();
}


float NodeBase::thetaFromMsg(const geometry_msgs::Quaternion &quat)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(quat, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::Quaternion NodeBase::msgFromTheta(float theta)
{
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    geometry_msgs::Quaternion result;
    tf::quaternionTFToMsg(q, result);
    return result;
}

void NodeBase::shutdown(const QString &reason)
{
    ROS_WARN_STREAM("Shutting down. Reason: " << reason.toStdString());
    ros::shutdown();
    settings().app->quit();
}

void NodeBase::reconfCb(const bigbang_eurobot::ReconfigureConstPtr &msg) 
{
    if (msg->target_node == ros::this_node::getName()) {
        ROS_WARN_STREAM("Node (" << ros::this_node::getName() << "): Received 'reconfigure' command!");
        updateParams();
    }
}

void NodeBase::drawSphere(const Coord_<float> &start, float radius, float dur, float r, float g, float b, float a)
{
    auto marker = Marker();
    marker.action = Marker::ADD;
    marker.type = Marker::SPHERE;
    marker.header.frame_id = baseFrameId().toStdString();
    marker.header.stamp = ros::Time::now();
    marker.id = m_lastMarker++;
    marker.header.seq = marker.id;
    marker.ns = nodeName().toStdString();
    marker.pose.position.x = start.getX();
    marker.pose.position.y = start.getY();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    m_markerPub.publish(marker);
    marker.pose.orientation = msgFromTheta(0);
    marker.lifetime = ros::Duration().fromSec(dur);
    m_markerPub.publish(marker);
}

void NodeBase::drawLine(const Coord_<float> &start, const Coord_<float> &end, float dur)
{
    auto marker = Marker();
    marker.action = Marker::ADD;
    marker.type = Marker::ARROW;
    marker.header.frame_id = baseFrameId().toStdString();
    marker.header.stamp = ros::Time::now();
    marker.id = m_lastMarker++;
    marker.header.seq = marker.id;
    marker.ns = nodeName().toStdString();
    marker.pose.position.x = start.getX();
    marker.pose.position.y = start.getY();
    marker.scale.x = start.dist_to(end);
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration().fromSec(dur);
    marker.pose.orientation = msgFromTheta(start.angle_from_x_axis_to(end));
    m_markerPub.publish(marker);
}
