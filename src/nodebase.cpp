#include "common/nodebase.hpp"
#include <QThread>

NodeBase::NodeBase() {
    thread()->setPriority(QThread::HighPriority);
    auto timer = new QTimer(this);
    timer->callOnTimeout(this, [this]{
        if (!ros::ok()) {
            shutdown();
        }
        ros::spinOnce();
    });
    timer->start(5);
}

void NodeBase::shutdown() {
    ros::shutdown();
    qApp->quit();
}