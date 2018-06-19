#include <cmath>
#include "ros/ros.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"

#include "drive_ros_obstacle_fusion/object.h"
#include "drive_ros_obstacle_fusion/cov_elements.h"
#include "drive_ros_msgs/ObstacleArray.h"
#include "drive_ros_msgs/tf2_drive_ros_msgs.h"

// parameter
std::string base_frame;
float dist_threshold;
float remove_trust; // in each cycle
int mov_avg_size;

// tf buffer
tf2_ros::Buffer* tf2_buffer;
tf2_ros::TransformListener* tf2_listener;

// covariances
Kalman::Covariance<Object::State> stateCov;
Kalman::Covariance<Object::State> procCov;

// object list and object publisher
std::vector<std::shared_ptr<Object>> obj_list;
ros::Publisher obj_pub;

// merges the new obstacle into the current obstacles or creates a new one
std::shared_ptr<Object> mergeObject(const drive_ros_msgs::Obstacle& o_msg)
{
    std::shared_ptr<Object> o_out = NULL;
    float min_dis = FLT_MAX;

    // check for nearest neighbour (brute force)
    for(auto o: obj_list)
    {
        auto s = o->getState();

        // euclidean distance
        auto dis = std::sqrt(static_cast<float>(
                             std::pow(s.x() - o_msg.centroid_pose.pose.position.x, 2) +
                             std::pow(s.y() - o_msg.centroid_pose.pose.position.y, 2)));

        if(dis < dist_threshold &&
           dis < min_dis)
        {
            o_out = o;
            min_dis = dis;
            ROS_DEBUG("found old object! x=%f y=%f trust=%f", o_out->getState().x(), o_out->getState().y(), o_out->getTrust());
        }
    }

    // no existing object found -> create new one
    if(NULL == o_out /*&& o_msg.trust > 0.3*/)
    {
        Object o;
        if(!o.init(stateCov, procCov,
                   o_msg.centroid_pose.pose.position.x,
                   o_msg.centroid_pose.pose.position.y,
                   o_msg.trust,
                   mov_avg_size))
        {
            ROS_ERROR_STREAM("Could not initilize object!");
            return NULL;
        }
        o_out = std::make_shared<Object>(o);
        obj_list.push_back(o_out);

        ROS_DEBUG_STREAM("created new object! Initial trust: " << o_out->getTrust());
    }

    return o_out;
}

// callback function for incoming prediction message
void predictionCallback(const drive_ros_msgs::ObstacleArrayConstPtr& msg)
{
    // process incoming obstacles
    for(auto ob: msg->obstacles)
    {
        // transform object
        drive_ros_msgs::Obstacle obs_trans;
        try {
            tf2_buffer->transform(ob, obs_trans, base_frame);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }

        // merge object
        ROS_DEBUG("Prediction! x=%f y=%f trust=%f", obs_trans.centroid_pose.pose.position.x,
                                                    obs_trans.centroid_pose.pose.position.y, obs_trans.trust);
        auto merged_ob = mergeObject(obs_trans);
        if(NULL == merged_ob) continue;

        // check timestamp
        if(ros::Time(0) == merged_ob->last_pred ||
           merged_ob->last_pred >= obs_trans.header.stamp)
        {
            merged_ob->last_pred     = obs_trans.header.stamp;
            merged_ob->last_corr     = obs_trans.header.stamp;
            merged_ob->last_centroid = obs_trans.centroid_pose.pose.position;
            continue;
        }

        // add trust to object
        merged_ob->addTrust(obs_trans.trust);

        // get differences
        float dt = (obs_trans.header.stamp - merged_ob->last_pred).toSec();
        float dx =  obs_trans.centroid_pose.pose.position.x - merged_ob->last_centroid.x;
        float dy =  obs_trans.centroid_pose.pose.position.y - merged_ob->last_centroid.y;
        ROS_DEBUG("dt=%f, dx=%f, dy=%f, vx=%f, vy=%f", dt, dx, dy, dx/dt, dy/dt);

        // predict the output
        merged_ob->predict(dt, dx/dt, dy/dt);
        ROS_DEBUG_STREAM("new state after predict: \n" << merged_ob->getState());

        // save time and centroid
        merged_ob->last_pred     = obs_trans.header.stamp;
        merged_ob->last_centroid = obs_trans.centroid_pose.pose.position;
    }

    // output objects
    drive_ros_msgs::ObstacleArray ob_array;
    for(auto ob = obj_list.begin(); ob != obj_list.end(); /* iteration in loop */ )
    {

        ROS_DEBUG_STREAM("obj: x=" << ob->get()->getState().x() <<
                            " y=" << ob->get()->getState().y() <<
                        " trust=" << ob->get()->getTrust());

        // remove if not enough trust
        if(0.001 > ob->get()->getTrust())
        {
            // delete object
            obj_list.erase(ob);

        }else{

            // publish
            drive_ros_msgs::Obstacle ob_out;

            ob_out.header.frame_id = base_frame;
            ob_out.header.stamp = ob->get()->last_pred;
            ob_out.obstacle_type = drive_ros_msgs::Obstacle::TYPE_GENERIC;
            ob_out.centroid_pose.pose.position.x = ob->get()->getState().x();
            ob_out.centroid_pose.pose.position.y = ob->get()->getState().y();
            ob_out.trust = ob->get()->getTrust();

            ob_array.obstacles.push_back(ob_out);

            // reduce trust
            ob->get()->substractTrust(remove_trust);

            // iterate to next element
            ob++;
        }
    }
    obj_pub.publish(ob_array);
}

// callback function for incoming correction messages
void correctionCallback(const drive_ros_msgs::ObstacleArrayConstPtr& msg)
{
    for(auto ob: msg->obstacles)
    {
        // transform object
        drive_ros_msgs::Obstacle obs_trans;
        try {
            tf2_buffer->transform(ob, obs_trans, base_frame);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }

        // merge object
        ROS_DEBUG("Correction! x=%f y=%f trust=%f", obs_trans.centroid_pose.pose.position.x,
                                                    obs_trans.centroid_pose.pose.position.y, obs_trans.trust);
        auto merged_ob = mergeObject(obs_trans);

        // no object found or created
        if(NULL == merged_ob) continue;

        // initial message -> prediction is needed first
        if(ros::Time(0) == merged_ob->last_corr) continue;

        // no prediction since last correction -> skip
        if(merged_ob->last_corr >= merged_ob->last_pred)
        {
            continue;
        }

        // check timestamp
        if(merged_ob->last_corr >= obs_trans.header.stamp)
        {
            merged_ob->last_corr = obs_trans.header.stamp;
            continue;
        }

        // get differences
        float dt = (obs_trans.header.stamp - merged_ob->last_pred).toSec();

        Kalman::Covariance<Object::Measurement> measCov;
        measCov.setZero();
        measCov(Object::Measurement::X, Object::Measurement::X) = obs_trans.centroid_pose.covariance[CovElem::lin_ang::linX_linX];
        measCov(Object::Measurement::X, Object::Measurement::Y) = obs_trans.centroid_pose.covariance[CovElem::lin_ang::linX_linY];
        measCov(Object::Measurement::Y, Object::Measurement::X) = obs_trans.centroid_pose.covariance[CovElem::lin_ang::linY_linX];
        measCov(Object::Measurement::Y, Object::Measurement::Y) = obs_trans.centroid_pose.covariance[CovElem::lin_ang::linY_linY];

        // do the correction
        merged_ob->correct(dt,
                           obs_trans.centroid_pose.pose.position.x,
                           obs_trans.centroid_pose.pose.position.y,
                           measCov);
        ROS_DEBUG_STREAM("new state after correct: \n" << merged_ob->getState());

        // remember last time
        merged_ob->last_corr = obs_trans.header.stamp;

        // add trust
        merged_ob->addTrust(obs_trans.trust);
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
    ret &= pnh.getParam("dist_threshold", dist_threshold);
    ret &= pnh.getParam("remove_trust", remove_trust);
    ret &= pnh.getParam("mov_avg_size", mov_avg_size);

    stateCov.setZero();
    procCov.setZero();
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

    // publisher
    obj_pub = pnh.advertise<drive_ros_msgs::ObstacleArray>("fused_obstacles", 1);

    // setup subscriber
    ros::Subscriber sub_pred = pnh.subscribe("prediction", 1, predictionCallback);
    ros::Subscriber sub_corr = pnh.subscribe("correction", 1, correctionCallback);

    ros::spin();

    return 0;
}
