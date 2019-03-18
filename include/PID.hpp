#pragma once


#include "FilteredController.hpp"
#include "SettingsPID.hpp"


namespace controller
{


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
    const Output & updateControl(Time time, const Input & ref, const Input & signal);
    void configureFirstRun(Time time, const Input &ref, const Input &signal);

    void mapFilterInputs(const Input &ref, const Input &signal, std::vector<Input> &input_filters);
    bool configureFilters(const std::vector<SettingsFilter> & settings);

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
