#pragma once


#include "FilteredController.hpp"


namespace controller
{


struct SettingsPID
{
    //! Proportional gain
    double kp;

    //! Integral gain
    double ki;

    //! Derivative gain
    double kd;

    //! Setpoint weighing term
    /** Setpoint weighing term, generally between 0 and 1
     * - B = 0 reference is introduced only through integral term
     * - B = 1 disables setpoint weighting
     */
    double weight_reference;

    //! Anti-integrator-windup
    /** Anti-integrator-windup time constant
     * - < 0 disable
     * - = 0 and Td = 0  disable
     * - = 0 and Td > 0  Tt = sqrt(Ti * Td)
     * */
    double gain_antiwidnup;

    SettingsPID();
};


class PID : public FilteredController
{
private:
    typedef typename Eigen::VectorXd Gain;

    bool antiwindup, mode_velocity_filtered, has_integral;

    Gain kp, ki, kd, gain_antiwidnup, weight_reference;

    Input dot_error;

    Output output_default, output;

    static bool nonnegative(const Gain & gain)
    {
        return (gain.array() >= 0).all();
    }

protected:
    const Output & updateControl(Time time, const Input & ref, const Input & signal, const Output & last_output);
    void configureFirstRun(Time time, const Input &ref, const Input &signal);

    void mapFilterInputs(const Input &ref, const Input &signal, std::vector<Input> &input_filters);

public:
    PID(unsigned int N_controllers);

    bool ok() const
    {
        return FilteredController::ok() &&
            nonnegative(kp) &&
            nonnegative(ki) &&
            nonnegative(kd) &&
            nonnegative(weight_reference) && (weight_reference.array() <= 1).all() &&
            nonnegative(gain_antiwidnup);
    }

    const Output & getDefaultOutput() const
    {
        return output_default;
    }

    bool configureFilters(const std::vector<SettingsFilter> & settings);

    bool configure(const std::vector<SettingsPID> & settings, double sampling);
    bool configure(const std::vector<SettingsPID> & settings, const SettingsFilter &settings_velocity_filter);

    void updateVelocities(const Input & dot_error);
};


}
