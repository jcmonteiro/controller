#pragma once

#include <vector>
#include "PID.hpp"

namespace pid_control
{

class PPI
{
public:
    //! Structure to hold the PPI parameters for 'PARALLEL' type PPI.
    struct ParallelSettings
    {
        //! Sampling time in seconds
        double Ts;

        //! Proportional gain of the external control loop
        double Kp1;

        //! Proportional gain of the internal control loop
        double Kp2;

        //! Integral gain
        double Ki;

        //! Derivative term
        /** Derivative term filtered by a first order system with time constant Td/N
         * - Typical values of N are between 8 and 20
         * - No derivative action on frequencies above N/Td
         */
        double N;

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
        ParallelSettings():Ts(0),Kp1(0),Kp2(0),Ki(0),B(1),Tt(-1),YMin(0),YMax(0){}

        bool operator==(const ParallelSettings &other) const{
            return
                Ts == other.Ts &&
                Kp1 == other.Kp1 &&
                Kp2 == other.Kp2 &&
                Ki == other.Ki &&
                B == other.B &&
                Tt == other.Tt &&
                YMin == other.YMin &&
                YMax == other.YMax;
        }

        bool operator!=(const ParallelSettings &other) const{
            return !(*this == other);
        }
    };

    struct Parameters
    {
        linear_system::IntegrationMethod integration_method;
        std::vector<ParallelSettings> p_pi_settings;
        bool wrap_2pi;
        bool saturate;

        Parameters() : integration_method(linear_system::TUSTIN), wrap_2pi(false), saturate(false) {}
        void setSettings(const std::vector<ParallelSettings> &settings) {p_pi_settings = settings;}
    };

    struct Inputs
    {
        linear_system::LinearSystem::Time time;
        Eigen::VectorXd reference, signal;
        Eigen::VectorXd dotreference, dotsignal;
        Eigen::VectorXd p_pi_sat;

        void checkDimensions(unsigned int dim) const;
        void resize(unsigned int dim);

        static Inputs create(double ref, double sig, double dref, double dsig, double sat);
    };

    struct Outputs
    {
        Eigen::VectorXd p_error, d_error;
        Eigen::VectorXd p_pi;

        void resize(unsigned int dim);
        const Eigen::VectorXd & getValue() {return p_pi;}
    };

private:
    unsigned int dim;

    PID pid;
    PID::Inputs pid_inputs;
    PID::Parameters pid_params;
    Parameters params;
    Outputs outputs;

public:
    void configParameters(const Parameters &params);
    inline void restart() {pid.restart();}
    const Outputs & update(const Inputs &inputs, bool check_dim = true);
};

}
