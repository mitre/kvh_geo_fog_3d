#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QHBoxLayout>
#include <unordered_map>
#include "diagnostic_msgs/DiagnosticArray.h"

#include "kvh_status_painter.hpp"

namespace kvh 
{

class StatusPanel : public rviz::Panel
{

Q_OBJECT
public:
    StatusPanel(QWidget* parent = 0);
    QHBoxLayout* StatusIndicatorFactory(bool, std::string, std::string);
    void DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr&);
protected:
    ros::NodeHandle nh_;
    std::unordered_map<std::string, StatusPainter*> painter_map_;
    ros::Subscriber diag_sub_;

};

}