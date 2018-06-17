#include <cmath>
#include "ros/ros.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"

#include "drive_ros_obstacle_fusion/moving_average.h"
#include "drive_ros_obstacle_fusion/object.h"
#include "drive_ros_msgs/ObstacleArray.h"
#include "drive_ros_msgs/tf2_drive_ros_msgs.h"

std::string base_frame = "";

float dis_treshold = 0;

float var_xx = 0, var_yy = 0;

tf2_ros::Buffer* tf2_buffer;
tf2_ros::TransformListener* tf2_listener;

Kalman::Covariance<Object::State> stateCov;
Kalman::Covariance<Object::State> procCov;

std::vector<std::shared_ptr<Object>> obj_list;

// merges the new obstacle into the current obstacles or creates a new one
std::shared_ptr<Object> mergeObject(const drive_ros_msgs::Obstacle& o_new)
{
    std::shared_ptr<Object> o_out = NULL;
    float min_dis = FLT_MAX;

    // check for nearest neighbour (brute force)
    for(auto o: obj_list)
    {
        auto s = o->getState();

        // euclidean distance
        auto dis = std::sqrt(static_cast<float>(
                             std::pow(s.x() - o_new.centroid.x, 2) +
                             std::pow(s.y() - o_new.centroid.y, 2)));

        if(dis < dis_treshold &&
           dis < min_dis)
        {
            o_out = o;
            min_dis = dis;
        }
    }

    // no existing object found -> create new
    if(NULL == o_out)
    {
        Object o;
        if(!o.init(stateCov, procCov))
        {
            ROS_ERROR_STREAM("Could not initilize object!");
            return NULL;
        }

        o_out = std::make_shared<Object>(o);
        obj_list.push_back(o_out);
    }

    return o_out;
}


void predictionCallback(const drive_ros_msgs::ObstacleArrayConstPtr& msg)
{
    for(auto ob: msg->obstacles)
    {
        // transform object
        drive_ros_msgs::Obstacle obs_trans;
        tf2_buffer->transform(ob, obs_trans, base_frame);

        // merge object
        auto old_ob = mergeObject(obs_trans);
        if(NULL == old_ob) break;

        // check timestamp
        if(ros::Time(0) == old_ob->last_pred ||
           old_ob->last_pred >= ob.header.stamp)
        {
            old_ob->last_pred = ob.header.stamp;
            old_ob->last_corr = ob.header.stamp;
            continue;
        }

        // get differences
        float dt = (ob.header.stamp - old_ob->last_pred).toSec();
        float dx = ob.centroid.x - old_ob->getState().x();
        float dy = ob.centroid.y - old_ob->getState().y();

        // predict the output
        old_ob->predict(dt, dx/dt, dy/dt);

        // save time
        old_ob->last_pred = ob.header.stamp;

    }
}


void correctionCallback(const drive_ros_msgs::ObstacleArrayConstPtr& msg)
{
    for(auto ob: msg->obstacles)
    {
        // transform object
        drive_ros_msgs::Obstacle obs_trans;
        tf2_buffer->transform(ob, obs_trans, base_frame);

        // merge object
        auto old_ob = mergeObject(obs_trans);

        // broken object -> bad
        if(NULL == old_ob) break;

        // initial message -> prediction is needed first
        if(ros::Time(0) == old_ob->last_corr) continue;

        // no prediction since last correction -> skip
        if(old_ob->last_corr >= old_ob->last_pred)
        {
            continue;
        }

        // check timestamp
        if(old_ob->last_corr >= ob.header.stamp)
        {
            old_ob->last_corr = ob.header.stamp;
            continue;
        }

        // get differences
        float dt = (ob.header.stamp - old_ob->last_pred).toSec();

        // do the correction
        old_ob->correct(dt, ob.centroid.x, ob.centroid.y, var_xx, var_yy);

        // remember last time
        old_ob->last_corr = ob.header.stamp;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_fusion");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // get parameter
    bool ret = true;
    ret &= pnh.getParam("base_frame", base_frame);
    ret &= pnh.getParam("dis_treshold", dis_treshold);

    procCov.setZero();
    stateCov.setZero();
    ret &= pnh.getParam("kalman_cov/meas_var_x", var_xx);
    ret &= pnh.getParam("kalman_cov/meas_var_y", var_yy);
    ret &= pnh.getParam("kalman_cov/filter_init_var_x", stateCov(Object::State::X, Object::State::X));
    ret &= pnh.getParam("kalman_cov/filter_init_var_y", stateCov(Object::State::Y, Object::State::Y));
    ret &= pnh.getParam("kalman_cov/sys_var_x", procCov(Object::State::X, Object::State::X));
    ret &= pnh.getParam("kalman_cov/sys_var_y", procCov(Object::State::Y, Object::State::Y));

    if(!ret)
    {
        ROS_ERROR("Problem occured while loading the parameters!");
        return 1;
    }

    // setup tf 2 stuff
    tf2_buffer =   new tf2_ros::Buffer();
    tf2_listener = new tf2_ros::TransformListener(*tf2_buffer);

    // setup subscriber
    ros::Subscriber sub_pred = nh.subscribe("prediction", 1, predictionCallback);
    ros::Subscriber sub_corr = nh.subscribe("correction", 1, correctionCallback);

    ros::spin();

    return 0;
}
