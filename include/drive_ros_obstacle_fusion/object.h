#ifndef OBJECT_H
#define OBJECT_H

#include <ros/ros.h>
#include "kalman/ExtendedKalmanFilter.hpp"
#include "object_system_model.h"
#include "object_measurement_model.h"


class Object
{
public:

    typedef float T;

    typedef Model::State<T> State;
    typedef Model::Control<T> Control;
    typedef Model::Measurement<T> Measurement;

    typedef Model::MeasurementModel<T> MeasurementModel;
    typedef Model::SystemModel<T> SystemModel;
    typedef Kalman::ExtendedKalmanFilter<State> Filter;

    Object(){};

    ros::Time last_pred;
    ros::Time last_corr;

    // initializes a new object
    bool init(const Kalman::Covariance<State>& stateCov,
              const Kalman::Covariance<State>& procCov)
    {
        bool ret = true;

        // init kalman
        State s;
        s.setZero();
        filter.init(s);

        meas_old.setZero();
        state_old.setZero();

        ros::Time last_pred(0);
        ros::Time last_corr(0);

        // Set initial state covariance
        ret &= filter.setCovariance(stateCov);

        // Set process noise covariance
        ret &= sys.setCovariance(procCov);

        return ret;
    }

    // predict state of object
    bool predict(const float dt, const float vx, const float vy)
    {
        // set control vector
        u.dt() = dt;
        u.vx() = vx;
        u.vy() = vy;

        // predict
        filter.predict(sys, u);

        // check if there is something wrong
        if(filter.getCovariance().hasNaN() ||
           filter.getState().hasNaN()       )
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
                 const float xx_var, const float yy_var)
    {
        // set measurement covariances
        Kalman::Covariance<Measurement> cov;
        cov.setZero();
        cov(Measurement::X, Measurement::X) = xx_var;
        cov(Measurement::Y, Measurement::Y) = yy_var;
        mm.setCovariance(cov);

        // set measurement vector
        z.x() = x - meas_old.x() + state_old.x();
        z.y() = y - meas_old.y() + state_old.y();

        // do the actual correction
        filter.update(mm, z);

        // check if there is something wrong
        if(cov.hasNaN())
        {
          ROS_ERROR("Measurement covariances are broken! Abort.");
          return false;
        }

        // save old values (to use differential measurements)
        state_old = filter.getState();
        meas_old.x() = x;
        meas_old.y() = y;

        return true;
    }

    // returns the current state
    State getState()
    {
        return filter.getState();
    }

private:

    // ros node handle
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    Control u;
    Measurement z;
    SystemModel sys;
    MeasurementModel mm;
    Filter filter;

    Measurement meas_old;
    State state_old;

};

#endif // OBJECT_H
