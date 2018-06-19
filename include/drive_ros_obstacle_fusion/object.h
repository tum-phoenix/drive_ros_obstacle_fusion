#ifndef OBJECT_H
#define OBJECT_H

#include <cmath>
#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "object_system_model.h"
#include "object_measurement_model.h"
#include "moving_average.h"


class Object
{
public:

    // kalman typedefs
    typedef float T;

    typedef Model::State<T> State;
    typedef Model::Control<T> Control;
    typedef Model::Measurement<T> Measurement;

    typedef Model::MeasurementModel<T> MeasurementModel;
    typedef Model::SystemModel<T> SystemModel;
    typedef Kalman::ExtendedKalmanFilter<State> Filter;

    // store some values
    ros::Time last_pred;
    ros::Time last_corr;
    geometry_msgs::Point last_centroid;

    // constructor
    Object(){};

    // initializes a new object
    bool init(const Kalman::Covariance<State>& stateCov,
              const Kalman::Covariance<State>& procCov,
              const float initial_x, const float initial_y,
              const float initial_trust, const int mov_avg_size)
    {
        bool ret = true;

        // init kalman
        State s;
        s.setZero();
        s.x() = initial_x;
        s.y() = initial_y;
        filter.init(s);

        // init trust
        trust = initial_trust;

        // init last values
        last_pred = ros::Time(0);
        last_corr = ros::Time(0);
        last_centroid.x = 0;
        last_centroid.y = 0;

        // init moving average
        vx_avg = new MovingAverage(mov_avg_size);
        vy_avg = new MovingAverage(mov_avg_size);

        // Set initial state covariance
        ret &= filter.setCovariance(stateCov);

        // Set process noise covariance
        ret &= sys.setCovariance(procCov);

        return ret;
    }

    // add some trust
    void addTrust(const float t)
    {
        trust += t;
        limitTrust();
    }

    // remove Trust
    void substractTrust(const float t)
    {
        trust -= t;
        limitTrust();
    }

    // get the current trust
    float getTrust(void)
    {
        return trust;
    }

    // predict state of object
    bool predict(const float dt, const float vx, const float vy)
    {
        // set control vector
        Control u;
        u.dt() = dt;
        u.vx() = vx_avg->addAndGetCrrtAvg(vx);
        u.vy() = vy_avg->addAndGetCrrtAvg(vy);

        // predict
        filter.predict(sys, u);

        // check if there is something wrong
        if(filter.getCovariance().hasNaN() ||
           filter.getState().hasNaN())
        {
          ROS_ERROR_STREAM("State covariances or vector is broken!" <<
                           "\nCovariances:\n" << filter.getCovariance() <<
                           "\nState:\n"       << filter.getState() <<
                           "\nInput:\n"       << u);
          return false;
        }

        return true;
    }

    // correct state of object
    bool correct(const float dt, const float x, const float y,
                 const Kalman::Covariance<Measurement>& measCov)
    {
        // set measurement covariances
        mm.setCovariance(measCov);

        // set measurement vector
        Measurement z;
        z.x() = x;
        z.y() = y;

        // do the actual correction
        filter.update(mm, z);

        // check if there is something wrong
        if(measCov.hasNaN())
        {
          ROS_ERROR("Measurement covariances are broken! Abort.");
          return false;
        }

        return true;
    }

    // returns the current state
    State getState()
    {
        return filter.getState();
    }

private:

    void limitTrust(void)
    {
        trust = std::min(1.0f, trust);
        trust = std::max(0.0f, trust);
    }

    // ros node handle (only for output)
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    // kalman filter stuff
    SystemModel sys;
    MeasurementModel mm;
    Filter filter;

    // current trust value
    float trust;

    // moving average
    MovingAverage* vx_avg;
    MovingAverage* vy_avg;

};

#endif // OBJECT_H
