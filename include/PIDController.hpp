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

    /**
     * @brief Generate a SettingsPID object to implement a PID with gains designed to produce the denominator of
     *
     * \f[
     *      \frac{u(s)}{e(s)} = \frac{\omega^2_n}{s^2 + 2\zeta\omega_n s + \omega^2_n}
     *      \times  \frac{r \omega_c}{s + r \omega_c }
     *      \quad ; \quad
     *      \omega_n = \frac{\omega_c}{\zeta}
     * \f]
     *
     * @param damping Damping coefficient \f$\zeta\f$.
     * @param cutoff Resonance frequency \f$\omega_c\f$.
     * @param far_pole_ratio Far pole ratio \f$r\f$ w.r.t. the cutoff frequency, typically between 4 and 10.
     * @return An instance of SettingsPID with kp, ki, and kd designed to meet the specifications.
     */
    static SettingsPID create(double damping, double cutoff, double far_pole_ratio);

    /**
     * @brief Generate a SettingsPID object to implement a PD with gains designed to produce the denominator of
     *
     * \f[
     *      \frac{u(s)}{e(s)} =  = \frac{\omega^2_n}{s^2 + 2\zeta\omega_n s + \omega^2_n}
     *      \quad ; \quad
     *      \omega_n = \frac{\omega_c}{\zeta}
     * \f]
     *
     * @param damping Damping coefficient \f$\zeta\f$.
     * @param cutoff Resonance frequency \f$\omega_c\f$.
     * @return An instance of SettingsPID with kp, ki = 0, and kd designed to meet the specifications.
     */
    static SettingsPID create(double damping, double cutoff);
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

    /**
     * @brief Calls #configure(const std::vector<SettingsPID> &, double) and configure every PID with \p settings.
     * @param settings Settings used for every PID channel.
     * @param sampling Controller sampling period.
     * @return
     */
    bool configure(const SettingsPID &settings, double sampling);

    /**
     * @brief Calls #configure(const std::vector<SettingsPID> &, const SettingsFilter &) and configure every PID with \p settings.
     * @param settings Settings used for every PID channel.
     * @param settings_velocity_filter Settings for the derivative filter.
     * @return
     */
    bool configure(const SettingsPID &settings, const SettingsFilter &settings_velocity_filter);

    void updateVelocities(const Input & dot_error);
};


}
