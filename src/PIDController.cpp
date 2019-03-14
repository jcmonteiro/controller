#include "PIDController.hpp"
#include <iostream>


using namespace controller;


SettingsPID::SettingsPID() :
    kp(0), ki(0), kd(0),
    weight_reference(1),
    gain_antiwidnup(0)
{
}

SettingsPID SettingsPID::create(double damping, double cutoff, double far_pole_ratio)
{
    SettingsPID ret;
    double wn = cutoff / damping;
    ret.kp = wn*wn * (1 + 2*damping*damping*far_pole_ratio);
    ret.ki = far_pole_ratio * damping * wn*wn*wn;
    ret.kd = damping * wn * (far_pole_ratio + 2);
    ret.gain_antiwidnup = 1 / ret.ki;
    ret.weight_reference = 1;
    return ret;
}

SettingsPID SettingsPID::create(double damping, double cutoff)
{
    SettingsPID ret;
    double wn = cutoff / damping;
    ret.kp = wn*wn;
    ret.ki = 0;
    ret.kd = 2 * damping * wn;
    ret.gain_antiwidnup = 0;
    ret.weight_reference = 1;
    return ret;
}


PID::PID(unsigned int N_controllers) :
    FilteredController(N_controllers, 2),
    antiwindup(true), mode_velocity_filtered(true),
    has_integral(true)
{
    kp.setZero(_N);
    ki.setZero(_N);
    kd.setZero(_N);
    weight_reference.setZero(_N);
    gain_antiwidnup.setZero(_N);
    output_default.setZero(_N);
}

const Output & PID::updateControl(Time time, const Input &ref, const Input &signal, const Output &last_output)
{
    FilteredController::updateControl(time, ref, signal, last_output);
    output = kp.cwiseProduct(weight_reference.cwiseProduct(ref) - signal)
               + kd.cwiseProduct( getFilters()[0].getOutput() )
               + getFilters()[1].getOutput();
    return output;
}

void PID::configureFirstRun(Time time, const Input &ref, const Input &signal)
{
    FilteredController::configureFirstRun(time, ref, signal);
}

void PID::mapFilterInputs(const Input &ref, const Input &signal, std::vector<Input> &input_filters)
{
    Input error = ref - signal;
    input_filters[0] = error;
    input_filters[1] = ki.cwiseProduct(error);
}

bool PID::configureFilters(const std::vector<SettingsFilter> & settings)
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

bool PID::configure(const std::vector<SettingsPID> &settings, double sampling)
{
    double max_cutoff = 0;
    for (const auto & s : settings)
    {
        if (s.kd <= 0)
            continue;
        double cutoff = s.kp / s.kd;
        if (cutoff > max_cutoff)
            max_cutoff = cutoff;
    }
    SettingsFilter velocity;
    if (max_cutoff == 0)
    {
        // if we are here, all kd <= 0 and there is no derivative action
        velocity.num.resize(1);
        velocity.den.resize(1);
        velocity.num << 0;
        velocity.den << 1;
        velocity.init_output_and_derivs.setZero(_N, 0);
    }
    else
    {
        // place the cutoff frequency two decades ahead
        max_cutoff *= 20;
        double damping = 0.9;
        double wn = max_cutoff / damping;
        velocity.num.resize(3);
        velocity.den.resize(3);
        velocity.num << 0, wn*wn, 0;
        velocity.den << 1, 2*damping*wn, wn*wn;
        velocity.init_output_and_derivs.setZero(_N, 2);
    }
    velocity.sampling_period = sampling;
    return configure(settings, velocity);
}

bool PID::configure(const std::vector<SettingsPID> &settings, const SettingsFilter &settings_velocity_filter)
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
    has_integral = false;
    for (auto setting : settings)
    {
        kp[k] = setting.kp;
        ki[k] = setting.ki;
        kd[k] = setting.kd;
        weight_reference[k] = setting.weight_reference;
        gain_antiwidnup[k]  = setting.gain_antiwidnup;
        ++k;

        has_integral |= setting.ki != 0;
    }

    SettingsFilter integrator;
    if (has_integral)
    {
        integrator.num.resize(2);
        integrator.den.resize(2);
        integrator.num << 0, 1;
        integrator.den << 1, 0;
        integrator.init_output_and_derivs.setZero(_N, 1);
    }
    else
    {
        integrator.num.resize(1);
        integrator.den.resize(1);
        integrator.num << 0;
        integrator.den << 1;
        integrator.init_output_and_derivs.setZero(_N, 0);
    }
    integrator.sampling_period = settings_velocity_filter.sampling_period;
    return configureFilters( {settings_velocity_filter, integrator} );
}

bool PID::configure(const SettingsPID &settings, double sampling)
{
    std::vector<SettingsPID> settings_vector( size() );
    for (auto &s : settings_vector)
        s = settings;
    return configure(settings_vector, sampling);
}

bool PID::configure(const SettingsPID &settings, const SettingsFilter &settings_velocity_filter)
{
    std::vector<SettingsPID> settings_vector( size() );
    for (auto &s : settings_vector)
        s = settings;
    return configure(settings_vector, settings_velocity_filter);
}

void PID::updateVelocities(const Input &dot_error)
{
    if (mode_velocity_filtered)
    {
        std::cerr << "[WARN] (PID::updateVelocities) Error velocity is computed inside the PID, so I refuse to directly updated it!" << std::endl;
        return;
    }
    this->dot_error = dot_error;
}
