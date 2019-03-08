#pragma once

#include <linear_system/LinearSystem.hpp>
#include "ParallelPIDSettings.hpp"

namespace pid_control
{

class PID
{
public:
    struct Parameters
    {
        linear_system::IntegrationMethod integration_method;
        std::vector<ParallelPIDSettings> pid_settings;
        bool wrap_2pi;
        bool saturate;

        Parameters() : integration_method(linear_system::IntegrationMethod::TUSTIN), wrap_2pi(false), saturate(false) {}
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
