#ifndef OBJECT_MEASUREMENT_MODEL_H
#define OBJECT_MEASUREMENT_MODEL_H

#include <kalman/LinearizedMeasurementModel.hpp>
#include "object_system_model.h"

namespace Model{

/**
 * @brief Measurement vector
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Measurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Measurement, T, 2)

    static constexpr size_t X = 0; //! x
    static constexpr size_t Y = 1; //! y

    T x()        const { return (*this)[ X ]; }
    T y()        const { return (*this)[ Y ]; }

    T& x()       { return (*this)[ X ]; }
    T& y()       { return (*this)[ Y ]; }
};

/**
 * @brief Measurement model
 *
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;

    //! Measurement type shortcut definition
    typedef Measurement<T> M;

    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;

        measurement.x() = x.x();
        measurement.y() = x.y();

        return measurement;
    }

protected:
    void updateJacobians( const S& x )
    {
        this->H.setZero();
        this->H(M::X, S::X) = 1;
        this->H(M::Y, S::Y) = 1;
    }
};

} // namespace


#endif // OBJECT_MEASUREMENT_MODEL_H
