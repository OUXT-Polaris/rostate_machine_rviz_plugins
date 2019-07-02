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

// Headers in STL
#include <memory>

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
    protected:
        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();
        virtual void processMessage(const rostate_machine::State::ConstPtr& msg);
        rviz::RosTopicProperty* topic_property_;
    private:
        ros::NodeHandle nh_;
        ros::Subscriber state_sub_;
        std::vector<std::string> split(const std::string &s,char delim);
        std::string convertToXmlParam(std::string state_topic);
        std::unique_ptr<StateMachine> state_machine_ptr_;
    };
}

#endif  //ROSTATE_MACHINE_RVIZ_PLUGINS_ROSTATE_MACHINE_VIEWER_DISPLAY_H_INCLUDED