#include "PIDController.hpp"
#include <iostream>


using namespace controller;


PID::PID(int N_controllers) :
    FilteredController(N_controllers),
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

const Controller::Output &PID::updateControl(Time time, const Input &ref, const Input &signal, const Output &last_output)
{
    auto error = ref - signal;
    updateFilters(time, {error, error});
    return last_output;
}

void PID::configureFirstRun(Time time, const Input &ref, const Input &signal)
{
    FilteredController::configureFirstRun(time, ref, signal);
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
    auto settings_copy = settings;
    for (auto & s : settings_copy)
        s.n_filters = _N;
    return FilteredController::configureFilters(settings_copy);
}

bool PID::configure(const std::vector<SettingsPID> &settings)
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

    return ok();
}

void PID::updateVelocities(const Controller::Input &dot_ref, const Controller::Input &dot_signal)
{
    if (mode_vel != Given)
    {
        std::cerr << "[WARN] (PID::updateVelocities) Velocity mode is not to Given, so I refuse to directly updated it!" << std::endl;
        return;
    }
    this->dot_ref = dot_ref;
    this->dot_signal = dot_signal;
}
