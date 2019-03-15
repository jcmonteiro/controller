#include "PIDController.hpp"
#include <iostream>
#include <cmath>


using namespace controller;


SettingsPID::SettingsPID() :
    far_pole(0),
    kp(0), ki(0), kd(0),
    weight_reference(1),
    gain_antiwidnup(0)
{
}

double SettingsPID::getSettlingTime()
{
    double damp = getDamping();
    // underdamped
    if (damp < 0.99)
        return -std::log(0.02*std::sqrt(1 - damp*damp))/damp/getNaturalFrequency();
    // critically damped
    else if (damp < 1.01)
        return 5.8 / getNaturalFrequency();
    // overdamped
    else if (damp < 1.1)
        return (5.8 + (damp - 1.01) * 1.1 / 0.09) / getNaturalFrequency();
    // "more" overdamped
    else if (damp < 5)
        return (6.9 + (damp - 1.1) * (38.8 - 6.9) / 4.9) / getNaturalFrequency();
    // "really" overdamped
    else
        return 38.8 / getNaturalFrequency();
}

double SettingsPID::getOvershoot()
{
    double damp = getDamping();
    // underdamped
    if (damp < 0.99)
        return std::exp( -damp * M_PI / std::sqrt(1 - damp*damp) );
    else
        return 0;
}

double SettingsPID::getDamping()
{
    // PD
    if (ki == 0)
        return kd / 2 / getNaturalFrequency();
    // PI
    else if (kd == 0)
        return kp / 2 / getNaturalFrequency();
    // PID
    else
    {
        return (kd - getFarPole()) / 2 / getNaturalFrequency();
    }
}

double SettingsPID::getFarPole()
{
    if (far_pole != 0)
        return far_pole;

    // otherwise, estimate the position
    if (ki > 0 && kd > 0)
    {
        double tmp = std::pow(ki/2 - (kd*kp)/6 + std::pow(kd , 3)/27 , 1.0/3.0);
        return kd/3 + tmp - (kp/3 - kd*kd/9) / tmp;
    }
    else
        return 0;
}

double SettingsPID::getNaturalFrequency()
{
    // PD
    if (ki == 0)
        return std::sqrt(kp);
    // PI
    else if (kd == 0)
        return std::sqrt(ki);
    // PID
    else
        return std::sqrt(ki / getFarPole());
}

double SettingsPID::getCutoffFrequency()
{
    return getDamping() * getNaturalFrequency();
}

double SettingsPID::getSuggestedSampling()
{
    double far_pole = getFarPole();
    double sampling;
    if (far_pole == 0 || far_pole < 1)
        sampling = M_PI_2 / getCutoffFrequency();
    else
        sampling = M_PI_2 / getCutoffFrequency() / far_pole;
    return std::min(sampling, getSettlingTime() / 100);
}

SettingsPID SettingsPID::createT(double overshoot, double settling_time)
{
    if (overshoot < 0)
        throw std::logic_error("[ERROR] (SettingsPID::createT) overshoot must lie in [0,1]");
    else if (overshoot > 1)
        throw std::logic_error("[ERROR] (SettingsPID::createT) overshoot must lie in [0,1]");
    if (settling_time <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createT) settling time must be positive");
    double damp, cutoff;
    if (overshoot > 0)
    {
        double tmp = std::log(overshoot);
        damp = -tmp / std::sqrt(M_PI*M_PI + tmp*tmp);
        cutoff = -std::log(0.02 * std::sqrt(1-damp*damp)) / settling_time;
    }
    else
    {
        damp = 1;
        cutoff = 5.8337 / settling_time;
    }
    return createF(damp, cutoff, 10);
}

SettingsPID SettingsPID::createF(double damping, double cutoff, double far_pole_ratio)
{
    if (damping <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createF) damping must lie in (0, 2]");
    else if (damping > 2)
        throw std::logic_error("[ERROR] (SettingsPID::createF) damping must lie in (0, 2]");
    if (cutoff <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createF) cutoff frequency must be positive");
    if (far_pole_ratio <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createF) far pole ratio must be positive");
    SettingsPID ret;
    double wn = cutoff / damping;
    ret.kp = wn*wn * (1 + 2*damping*damping*far_pole_ratio);
    ret.ki = far_pole_ratio * damping * wn*wn*wn;
    ret.kd = damping * wn * (far_pole_ratio + 2);
    ret.gain_antiwidnup = 1 / ret.ki;
    ret.weight_reference = 1;
    ret.far_pole = far_pole_ratio * damping * wn;
    return ret;
}

SettingsPID SettingsPID::createF(double damping, double cutoff)
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
               + kd.cwiseProduct( mode_velocity_filtered ? getFilters()[0].getOutput() : this->dot_error )
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
