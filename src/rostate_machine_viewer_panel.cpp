/**
 * @file rostate_machine_viewer_panel.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of RostateMachineViewerPanel class
 * @version 0.1
 * @date 2019-07-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <rostate_machine_rviz_plugins/rostate_machine_viewer_panel.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

#include <memory>

namespace rostate_machine_rviz_plugins
{
    RostateMachineViewerPanel::RostateMachineViewerPanel(QWidget* parent)
    {
        QVBoxLayout* layout = new QVBoxLayout;

        QHBoxLayout* topic_layout = new QHBoxLayout;
        topic_layout->addWidget(new QLabel("State Topic:"));
        QStringList topic_lists = updateTopicInfo();
        state_topic_combo_ = new QComboBox;
        state_topic_combo_->addItems(topic_lists);
        topic_layout->addWidget(state_topic_combo_);
        layout->addLayout(topic_layout);

        QVBoxLayout* state_view_layout = new QVBoxLayout;
        state_view_ = new QGraphicsView;
        state_view_layout->addWidget(state_view_);
        state_view_layout->addWidget(new QLabel("State"));
        layout->addLayout(state_view_layout);

        setLayout(layout);
        connect(state_topic_combo_, SIGNAL(activated(QString)), this, SLOT(updateTopic(QString)));
    }

    QStringList RostateMachineViewerPanel::updateTopicInfo()
    {
        QStringList ret;
        std::vector<ros::master::TopicInfo> topic_info;
        ros::master::getTopics(topic_info);
        for(auto itr = topic_info.begin(); itr != topic_info.end(); itr++)
        {
            if(itr->datatype == "rostate_machine/State")
            {
                QString topic_str = QString::fromUtf8(itr->name.c_str());
                ret.append(topic_str);
            }
        }
        return ret;
    }

    std::string RostateMachineViewerPanel::convertToDotStringTopic(std::string state_topic)
    {
        std::string ret;
        std::vector<std::string> str_vec = split(state_topic,'/');
        for(int i=0; i<str_vec.size()-1; i++)
        {
            ret = ret + str_vec[i] + "/";
        }
        ret = "/" + ret + "dot_string";
        return ret;
    }

    std::vector<std::string> RostateMachineViewerPanel::split(const std::string &s,char delim)
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

    void RostateMachineViewerPanel::load( const rviz::Config& config )
    {

    }

    void RostateMachineViewerPanel::save( rviz::Config config ) const
    {
        rviz::Panel::save(config);
    }

    void RostateMachineViewerPanel::updateTopic(QString text)
    {
        dot_string_sub_.shutdown();
        std::string state_topic = text.toUtf8().constData();
        dot_string_topic_ = convertToDotStringTopic(state_topic);
        dot_string_sub_ = nh_.subscribe(dot_string_topic_,1,&RostateMachineViewerPanel::dotStringCallback,this);
    }

    void RostateMachineViewerPanel::dotStringCallback(const std_msgs::String::ConstPtr msg)
    {
        mtx_.lock();
        dot_string_ = msg->data;
        mtx_.unlock();
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rostate_machine_rviz_plugins::RostateMachineViewerPanel,rviz::Panel)