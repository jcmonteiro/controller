#pragma once


#include "FilteredController.hpp"


namespace controller
{


class SettingsPID
{
private:
    double far_pole;

public:
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

    /**
     * @brief Default constructor.
     */
    SettingsPID();

    /**
     * @brief Returns the expected settling time given the PID settings.
     * @return The expected settling time.
     */
    double getSettlingTime();

    /**
     * @brief Returns the expected overshoot given the PID settings.
     *
     * The overshoot is returned [0, 1], e.g. 0.1 corresponds to 10% overshoot.
     *
     * @return The expected overshoot.
     */
    double getOvershoot();

    /**
     * @brief Returns the expected damping given the PID settings.
     *
     * @return The expected damping.
     */
    double getDamping();

    /**
     * @brief Returns the far pole position (only valid for kp,ki,kd > 0).
     * @return The far pole position.
     */
    double getFarPole();

    /**
     * @brief Returns the expected natural frequency (in rad) given the PID settings.
     *
     * @return The expected natural frequency.
     */
    double getNaturalFrequency();

    /**
     * @brief Returns the expected cutoff frequency (in rad) given the PID settings.
     * @return The expected cutoff frequency
     */
    double getCutoffFrequency();

    /**
     * @brief Returns the suggested sampling, computed as min(0.25 * 2*pi/w_max, ts/100),
     * where w_max is the fastest pole produced by this PID settings and ts = #getSamplingTime().
     * @return The suggested sampling.
     */
    double getSuggestedSampling();

    /**
     * @brief Generate a SettingsPID instance with gains designed to meet the provided time specifications.
     * @param overshoot The maximum allowed overshoot, typically [0, 0.2]. Accepted values are in [0, 1].
     * @param settling_time The desired settling time, for which the step-response error < 2%.
     * @return An instance of SettingsPID with kp, ki, and kd designed to meet the specifications.
     * @throws std::logic_error If parameters are not in a valid range.
     */
    static SettingsPID createT(double overshoot, double settling_time);

    /**
     * @brief Generate a SettingsPID instance to implement a PID with gains designed to produce the denominator of
     *
     * \f[
     *      \frac{u(s)}{e(s)} = \frac{\omega^2_n}{s^2 + 2\zeta\omega_n s + \omega^2_n}
     *      \times  \frac{r \omega_c}{s + r \omega_c }
     *      \quad ; \quad
     *      \omega_n = \frac{\omega_c}{\zeta}
     * \f]
     *
     * A general rule of thumb for choosing the cutoff frequency \f$\omega_c\f$, if you don't wish to set
     * it based on frequency analysis, is to set it to \f$ \omega_c = 4\zeta t_s \f$, where \f$ t_s \f$ corresponds
     * to the desired settling time.
     *
     * This choice ensures step-response error < 2% for t >\f$t_s\f$.
     *
     * @param damping Damping coefficient \f$\zeta\f$, valid values are in (0, 2].
     * @param cutoff Cutoff frequency \f$\omega_c\f$, valid values are > 0.
     * @param far_pole_ratio Far pole ratio \f$r\f$ w.r.t. the cutoff frequency, typically between 4 and 10.
     * Valid values are > 0.
     * @return An instance of SettingsPID with kp, ki, and kd designed to meet the specifications.
     * @throws std::logic_error If parameters are not in a valid range.
     */
    static SettingsPID createF(double damping, double cutoff, double far_pole_ratio);

    /**
     * @brief Generate a SettingsPID instance to implement a PD with gains designed to produce the denominator of
     *
     * \f[
     *      \frac{u(s)}{e(s)} =  = \frac{\omega^2_n}{s^2 + 2\zeta\omega_n s + \omega^2_n}
     *      \quad ; \quad
     *      \omega_n = \frac{\omega_c}{\zeta}
     * \f]
     *
     * A general rule of thumb for choosing the cutoff frequency \f$\omega_c\f$, if you don't wish to set
     * it based on frequency analysis, is to set it to \f$ \omega_c = 4\zeta t_s \f$, where \f$ t_s \f$ corresponds
     * to the desired settling time.
     *
     * This choice ensures step-response error < 2% for t >\f$t_s\f$._s\f$ and error < 1% for t >\f$t_s\f$.
     *
     * @param damping Damping coefficient \f$\zeta\f$, valid values are in (0, 2].
     * @param cutoff Cutoff frequency \f$\omega_c\f$, valid values are > 0.
     * @return An instance of SettingsPID with kp, ki = 0, and kd designed to meet the specifications.
     * @throws std::logic_error If parameters are not in a valid range.
     */
    static SettingsPID createF(double damping, double cutoff);
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
