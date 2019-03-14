#include "PIDController.hpp"
#include <iostream>


using namespace controller;


SettingsPID::SettingsPID() :
    kp(0), ki(0), kd(0),
    weight_reference(1),
    gain_antiwidnup(0)
{
}


PID::PID(unsigned int N_controllers) :
    FilteredController(N_controllers, 2),
    mode_vel(Filtered),
    antiwindup(true)
{
    kp.setZero(_N);
    ki.setZero(_N);
    kd.setZero(_N);
    weight_reference.setZero(_N);
    gain_antiwidnup.setZero(_N);
    output_default.setZero(_N);
}

const Input &PID::updateControl(Time time, const Input &ref, const Input &signal, const Output &last_output)
{
    FilteredController::updateControl(time, ref, signal, last_output);
    return last_output;
}

void PID::configureFirstRun(Time time, const Input &ref, const Input &signal)
{
    FilteredController::configureFirstRun(time, ref, signal);
}

void PID::mapFilterInputs(const Input &ref, const Input &signal, std::vector<Input> &input_filters)
{
    Input error = ref - signal;
    input_filters[0] = error;
    input_filters[1] = error;
}

bool PID::configureFilters(const std::vector<SettingsFilters> & settings)
{
    if (settings.size() != 2)
    {
        std::cerr << "[WARN] (PID::configureFilters) PID must be configured for two filters, not "
                  << settings.size() << ". All filters will be removed." << std::endl;
        FilteredController::configureFilters({});
        return false;
    }
    return FilteredController::configureFilters(settings);
}

bool PID::configure(const std::vector<SettingsPID> &settings, const std::vector<SettingsFilters> &settings_filters)
{
    if (settings.size() != _N)
    {
        if (settings.size() > _N)
            std::cerr << "[Warn] (PID::configure) failed to configure, there are too many settings" << std::endl;
        else
            std::cerr << "[Warn] (PID::configure) failed to configure, there are not enough settings" << std::endl;
        return false;
    }

    unsigned int k = 0;
    for (auto setting : settings)
    {
        kp[k] = setting.kp;
        ki[k] = setting.ki;
        kd[k] = setting.kd;
        weight_reference[k] = setting.weight_reference;
        gain_antiwidnup[k]  = setting.gain_antiwidnup;
        ++k;
    }

    return configureFilters(settings_filters);
}

void PID::updateVelocities(const Input &dot_ref, const Input &dot_signal)
{
    if (mode_vel != Given)
    {
        std::cerr << "[WARN] (PID::updateVelocities) Velocity mode is not to Given, so I refuse to directly updated it!" << std::endl;
        return;
    }
    this->dot_ref = dot_ref;
    this->dot_signal = dot_signal;
}
