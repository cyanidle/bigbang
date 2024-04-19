#pragma once



#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "bigbang/types/costmap.hpp"
#include "ros/ros.h"
#include "bigbang/types/rosparams.h"
#include "std_msgs/String.h"
#include <type_traits>
#include "bigbang_eurobot/Reconfigure.h"
#include <QCoreApplication>
#include <QTimer>
#include <tf/transform_broadcaster.h>

struct NodeSettings
{
    QCoreApplication* app;
    QString node_name;
    int spin_rate_ms;
};

class NodeBase : public QObject
{
    Q_OBJECT
public:
    NodeBase(const NodeSettings &settings);
    const QString &nodeName() const {return m_settings.node_name;}
    const NodeSettings &settings() {return m_settings;}
    ros::NodeHandle* nh() {return &m_nh;}
    virtual ~NodeBase();
    static float thetaFromMsg(const geometry_msgs::Quaternion &quat);
    static geometry_msgs::Quaternion msgFromTheta(float theta);
    virtual const QString &baseFrameId() const = 0;
    void shutdown(const QString &reason = "Not Given");

    void drawLine(const Coord_<float> &start, const Coord_<float> &end, float dur = 0.5);
    void drawSphere(const Coord_<float> &start, float radius = 0.2, float dur = 0.5, float r = 1, float g = 0, float b = 0, float a = 1);
public slots:
    virtual void updateParams(const QString &name = "") = 0;
private slots:
    void watchdog();
private:
    void reconfCb(const bigbang_eurobot::ReconfigureConstPtr &msg);
    std_msgs::String m_stringMsg;
    ros::Subscriber m_reconfSub;
    ros::Publisher m_markerPub;
    NodeSettings m_settings;
    ros::NodeHandle m_nh;
    QTimer* m_watchdog;
    quint32 m_lastMarker{0};
};

template<typename Node, typename...Args>
int node_main(int argc, char* argv[], const char* node_name, Args&&...args) {
    static_assert(std::is_base_of<NodeBase, Node>::value,
         "Node must inherit from NodeBase!");
    QCoreApplication app(argc, argv);
    ros::init(argc, argv, node_name);
    Node nodeObj {NodeSettings{&app, node_name, 5}, std::forward<Args>(args)...};
    Q_UNUSED(nodeObj)
    return app.exec();
}


#define NODE_MAIN(node, node_name) \
int main(int argc, char* argv[]) { \
    return node_main<node>(argc, argv, node_name); \
}



