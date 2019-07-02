/**
 * @file rostate_machine_viewer_panel.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Definition of RostateMachineViewerPanel class
 * @version 0.1
 * @date 2019-07-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef ROSTATE_MACHINE_RVIZ_PLUGINS_ROSTATE_MACHINE_VIEWER_PANEL_H_INCLUDED
#define ROSTATE_MACHINE_RVIZ_PLUGINS_ROSTATE_MACHINE_VIEWER_PANEL_H_INCLUDED

#ifndef Q_MOC_RUN
    #include <ros/ros.h>
    #include <rviz/panel.h>
#endif

// Headers in STL
#include <memory>
#include <mutex>
#include<iostream>
#include<fstream>

// Headers in Qt
#include <QComboBox>
#include <QImage>
#include <QGraphicsView>
#include <QGraphicsScene>

// Headers in ROS
#include <ros/master.h>
#include <std_msgs/String.h>

// Headers in Boost
#include <boost/thread.hpp>
#include <boost/optional.hpp>

namespace rostate_machine_rviz_plugins
{
    class RostateMachineViewerPanel : public rviz::Panel
    {
    Q_OBJECT
    public:
        RostateMachineViewerPanel(QWidget* parent = 0 );
        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;
    protected Q_SLOTS:
        void updateTopic(QString text);
    protected:
        QComboBox state_topic_combo_;
        QGraphicsView state_view_;
        QGraphicsScene Scene_;
    private:
        boost::thread view_update_thread_;
        void updateStateView();
        QStringList updateTopicInfo();
        ros::Subscriber dot_string_sub_;
        void dotStringCallback(const std_msgs::String::ConstPtr msg);
        ros::NodeHandle nh_;
        std::string dot_string_topic_;
        std::vector<std::string> split(const std::string &s,char delim);
        std::string convertToDotStringTopic(std::string state_topic);
        std::string dot_string_;
        std::mutex mtx_;
        boost::optional<QImage> dot_image_;
    };
}

#endif  //ROSTATE_MACHINE_RVIZ_PLUGINS_ROSTATE_MACHINE_VIEWER_PANEL_H_INCLUDED