#pragma once


#include "FilteredController.hpp"
#include "SettingsPID.hpp"


namespace controller
{


class PID : public FilteredController
{
private:
    typedef typename Eigen::VectorXd Gain;

    bool antiwindup, mode_velocity_filtered, has_integral, has_derivative;

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

    void mapFilterInputs(const Input &ref, const Input &signal, std::vector<linear_system::Input> &input_filters);
    void mapInitialOutputAndDerivatives(std::vector<Eigen::MatrixXd> &initial_out_dout);
    bool configureFilters(const std::vector<SettingsFilter> & settings);

public:
    PID(unsigned int N_controllers);

    /**
     * @brief Returns true if everything is correctly configured.
     * @return True if configuration is ok.
     */
    bool ok() const
    {
        return FilteredController::ok() &&
            nonnegative(kp) &&
            nonnegative(ki) &&
            nonnegative(kd) &&
            nonnegative(weight_reference) && (weight_reference.array() <= 1).all() &&
            nonnegative(gain_antiwidnup);
    }

    /**
     * @brief Returns the default output for this PID instance.
     * @return The default output.
     */
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
     * @return True if configuration succeeds.
     */
    bool configure(const SettingsPID &settings, double sampling);

    /**
     * @brief Calls #configure(const std::vector<SettingsPID> &, const SettingsFilter &) and configure every PID with \p settings.
     * @param settings Settings used for every PID channel.
     * @param settings_velocity_filter Settings for the derivative filter.
     * @return True if configuration succeeds.
     */
    bool configure(const SettingsPID &settings, const SettingsFilter &settings_velocity_filter);

    /**
     * @brief Returns true if the error derivative is computed internally using some lead filter.
     * @return True if the derivative is filtered. False if it is given by calling #setErrorDerivative.
     * @see #setDerivativeFiltered, #setErrorDerivative
     */
    inline bool isDerivativeFiltered() const
    {
        return mode_velocity_filtered;
    }

    /**
     * @brief Sets the derivative mode.
     * @param is_filtered True to compute the derivative internally using some lead filter. False
     * to acquire it through #setErrorDerivative.
     * @see #isDerivativeFiltered, #setErrorDerivative
     */
    inline void setDerivativeFiltered(bool is_filtered)
    {
        mode_velocity_filtered = is_filtered;
    }

    /**
     * @brief Returns the error derivative.
     *
     * By default, this method returns current veleocity filter output, unless #isDerivativeFiltered()
     * == false, then it returns the last value set via #setErrorDerivative.
     *
     * @return The error derivative.
     * @see #setErrorDerivative, #isDerivativeFiltered
     */
    const Eigen::VectorXd & getErrorDerivative() const;

    /**
     * @brief Updates the error derivative.
     *
     * A call to this method only takes effect if #isDerivativeFiltered() == false.
     *
     * @param dot_error The error derivative.
     * @see #isDerivativeFiltered, #setDerivativeFiltered, #getErrorDerivative
     */
    void setErrorDerivative(const Input & dot_error);
};


}
