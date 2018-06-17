#ifndef OBJECT_SYSTEM_MODEL_H
#define OBJECT_SYSTEM_MODEL_H

#include <kalman/LinearizedSystemModel.hpp>

namespace Model{

/**
 * @brief System state vector-type for motion model
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(State, T, 4)

    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! X-velocity
    static constexpr size_t VX = 2;
    //! Y-velocity
    static constexpr size_t VY = 3;

    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T vx()      const { return (*this)[ VX ]; }
    T vy()      const { return (*this)[ VY ]; }

    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& vx()     { return (*this)[ VX ]; }
    T& vy()     { return (*this)[ VY ]; }
};

/**
 * @brief System control-input vector-type for motion model
 *
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(Control, T, 3)

    static constexpr size_t VX = 0;
    static constexpr size_t VY = 1;
    static constexpr size_t DT = 2;

    T vx()      const { return (*this)[ VX ]; }
    T vy()      const { return (*this)[ VY ]; }
    T dt()      const { return (*this)[ DT ]; }

    T& vx()     { return (*this)[ VX ]; }
    T& vy()     { return (*this)[ VY ]; }
    T& dt()     { return (*this)[ DT ]; }
};

/**
 * @brief System model
 *
 * This is the system model defining how the obstacle moves from one
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;

    //! Control type shortcut definition
    typedef Control<T> C;

    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] s The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& s, const C& u) const
    {
        //! Predicted state vector after transition
        S s_;

        s_.x() = s.x() + s.vx() * u.dt();
        s_.y() = s.y() + s.vy() * u.dt();
        s_.vx() = u.vx();
        s_.vy() = u.vy();

        // Return transitioned state vector
        return s_;
    }
protected:
    void updateJacobians( const S& s, const C& u )
    {
        this->F.setIdentity();
        this->F(S::X, S::VX) = u.dt();
        this->F(S::Y, S::VY) = u.dt();
    }
};

} // namespace

#endif // OBJECT_SYSTEM_MODEL_H
