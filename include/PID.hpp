#pragma once

#include <linear_system/LinearSystem.hpp>

namespace controller
{

class PID
{
public:
    //! Structure to hold the PID parameters for 'PARALLEL' type PID.
    struct ParallelSettings
    {
        //! Sampling time in seconds
        double Ts;

        //! Proportional gain
        double Kp;

        //! Integral gain
        double Ki;

        //! Derivative gain
        double Kd;

        //! Setpoint weighing term
        /** Setpoint weighing term, generally between 0 and 1
         * - B = 0 reference is introduced only through integral term
         * - B = 1 disables setpoint weighting
         */
        double B;

        //! Anti-integrator-windup
        /** Anti-integrator-windup time constant
         * - < 0 disable
         * - = 0 and Td = 0  disable
         * - = 0 and Td > 0  Tt = sqrt(Ti * Td)
         * */
        double Tt;

        //! Minimum output value
        double YMin;
        //! Maximum output value
        double YMax;

        //! Constructor
        ParallelSettings():Ts(0),Kp(0),Ki(0),Kd(0),B(1),Tt(-1),YMin(0),YMax(0){}

        bool operator==(const ParallelSettings &other) const{
            return
                Ts == other.Ts &&
                Kp == other.Kp &&
                Ki == other.Ki &&
                Kd == other.Kd &&
                B == other.B &&
                Tt == other.Tt &&
                YMin == other.YMin &&
                YMax == other.YMax;
        }

        bool operator!=(const ParallelSettings &other) const{
            return !(*this == other);
        }

        void setIdealCoefficients(
            double _K = 0,
            double _Ti = 0,
            double _Td = 0);
    };

    struct Parameters
    {
        linear_system::IntegrationMethod integration_method;
        std::vector<ParallelSettings> pid_settings;
        bool wrap_2pi;
        bool saturate;

        Parameters() : integration_method(linear_system::IntegrationMethod::TUSTIN), wrap_2pi(false), saturate(false) {}
        void setSettings(const std::vector<ParallelSettings> &settings) {pid_settings = settings;}
    };

    struct Inputs
    {
        linear_system::LinearSystem::Time time;
        Eigen::VectorXd reference, signal;
        Eigen::VectorXd dotreference, dotsignal;
        Eigen::VectorXd pid_sat;

        void checkDimensions(unsigned int dim) const;
        void resize(unsigned int dim);

        static Inputs create(double ref, double sig, double dref, double dsig, double sat);
    };

    struct Outputs
    {
        Eigen::VectorXd p_error, d_error;
        Eigen::VectorXd p, i, d, pid;

        void resize(unsigned int dim);
        const Eigen::VectorXd & getValue() {return pid;}
    };

private:
    Parameters params;
    Outputs outputs;

    bool first_run, has_integrator;
    unsigned int dim;
    Eigen::VectorXd p_error_with_B, integrator_input;
    Eigen::VectorXd B, Kp, Kd, Ki, Kt;

    linear_system::LinearSystem integrator;

    void configIntegrator();

public:
    PID() : first_run(true), has_integrator(false), dim(0) {}
    void configParameters(const Parameters &params);

    /*!
     * \brief restart Schedules a restart for the next update to reset the integrator
     */
    inline void restart() {first_run = true;}

    /*!
     * \brief cancelRestart Cancels a scheduled restart
     */
    inline void cancelRestart() {first_run = false;}

    const Outputs & update(const Inputs & inputs, bool check_dim = true);
};

}
