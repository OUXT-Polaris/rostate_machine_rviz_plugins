/**
 * @file rostate_machine_viewer_display.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Definition of the RostateMachineViewerDisplay class
 * @version 0.1
 * @date 2019-07-03
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef ROSTATE_MACHINE_RVIZ_PLUGINS_ROSTATE_MACHINE_VIEWER_DISPLAY_H_INCLUDED
#define ROSTATE_MACHINE_RVIZ_PLUGINS_ROSTATE_MACHINE_VIEWER_DISPLAY_H_INCLUDED

#ifndef Q_MOC_RUN
    // Headers in Rviz
    #include <rviz/render_panel.h>
    #include <rviz/image/image_display_base.h>
#endif

// Headers in ROS
#include <ros/ros.h>
#include <rostate_machine/state_machine.h>
#include <rostate_machine/State.h>
#include <jsk_rviz_plugins/overlay_utils.h>

// Headers in Rviz
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>

// Headers in STL
#include <memory>

// Headers in Boost
#include <boost/circular_buffer.hpp>

// Headers in OpenCV
#include <opencv2/opencv.hpp>

namespace rostate_machine_rviz_plugins
{
    class RostateMachineViewerDisplay:public rviz::Display
    {
    Q_OBJECT
    public:
        RostateMachineViewerDisplay();
        virtual ~RostateMachineViewerDisplay();
        virtual void onInitialize();
        virtual void update( float wall_dt, float ros_dt );
        virtual void reset();
    protected Q_SLOTS:
        virtual void updateTopic();
        virtual void updateTop();
        virtual void updateLeft();
        virtual void updateAlpha();
    protected:
        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();
        virtual void processMessage(const rostate_machine::State::ConstPtr& msg);
        rviz::RosTopicProperty* topic_property_;
        rviz::IntProperty* left_property_;
        rviz::IntProperty* top_property_;
        rviz::FloatProperty* alpha_property_;
        int left_, top_;
        double alpha_;
    private:
        ros::NodeHandle nh_;
        ros::Subscriber state_sub_;
        std::vector<std::string> split(const std::string &s,char delim);
        std::string convertToXmlParam(std::string state_topic);
        std::unique_ptr<StateMachine> state_machine_ptr_;
        jsk_rviz_plugins::OverlayObject::Ptr overlay_;
        bool is_msg_available_;
        bool require_update_;
        boost::mutex mutex_;
        boost::circular_buffer<std::string> state_buf_;
        cv::Mat generateGraphImage();
    };
}

#endif  //ROSTATE_MACHINE_RVIZ_PLUGINS_ROSTATE_MACHINE_VIEWER_DISPLAY_H_INCLUDED