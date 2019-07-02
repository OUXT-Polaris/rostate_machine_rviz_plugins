#include <rostate_machine_rviz_plugins/rostate_machine_viewer_display.h>

namespace rostate_machine_rviz_plugins
{
    RostateMachineViewerDisplay::RostateMachineViewerDisplay()
    {
        topic_property_ = new rviz::RosTopicProperty("State Topic", "",
            QString::fromStdString(ros::message_traits::datatype<rostate_machine::State>()),
            "rostate_machine::State topic to subscribe to.", this, SLOT(updateTopic()));
    }

    RostateMachineViewerDisplay::~RostateMachineViewerDisplay()
    {

    }

    void RostateMachineViewerDisplay::onInitialize()
    {

    }

    void RostateMachineViewerDisplay::update( float wall_dt, float ros_dt )
    {

    }

    void RostateMachineViewerDisplay::reset()
    {

    }

    void RostateMachineViewerDisplay::onEnable()
    {

    }

    void RostateMachineViewerDisplay::onDisable()
    {

    }

    std::vector<std::string> RostateMachineViewerDisplay::split(const std::string &s,char delim)
    {
        std::vector<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (getline(ss, item, delim))
        {
            if (!item.empty())
            {
                elems.push_back(item);
            }
        }
        return elems;
    }

    std::string RostateMachineViewerDisplay::convertToXmlParam(std::string state_topic)
    {
        std::string ret;
        std::vector<std::string> str_vec = split(state_topic,'/');
        for(int i=0; i<str_vec.size()-1; i++)
        {
            ret = ret + str_vec[i] + "/";
        }
        ret = "/" + ret + "description";
        return ret;
    }

    void RostateMachineViewerDisplay::processMessage(const rostate_machine::State::ConstPtr& msg)
    {
        state_machine_ptr_->setCurrentState(msg->current_state);
    }

    void RostateMachineViewerDisplay::updateTopic()
    {
        std::string xml_param = convertToXmlParam(topic_property_->getTopicStd());
        std::string description;
        nh_.param<std::string>(xml_param, description, "");
        state_machine_ptr_ = std::unique_ptr<StateMachine>(new StateMachine(description));
        state_sub_.shutdown();
        state_sub_ = nh_.subscribe(topic_property_->getTopicStd(),1,&RostateMachineViewerDisplay::processMessage,this);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rostate_machine_rviz_plugins::RostateMachineViewerDisplay, rviz::Display)