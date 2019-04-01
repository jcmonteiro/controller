#include "PID.hpp"
#include <iostream>


using namespace controller;


PID::PID(unsigned int N_controllers) :
    FilteredController(N_controllers, 2),
    antiwindup(true), mode_velocity_filtered(true),
    has_integral(false), has_derivative(false)
{
    kp.setZero(_N);
    ki.setZero(_N);
    kd.setZero(_N);
    weight_reference.setZero(_N);
    gain_antiwidnup.setZero(_N);
    dot_error_zero.setZero();
}

const Output & PID::updateControl(Time time, const Input &ref, const Input &signal)
{
    FilteredController::updateControl(time, ref, signal);
    output = kp.cwiseProduct(weight_reference.cwiseProduct(ref) - signal);
    if (has_derivative)
    {
        output += kd.cwiseProduct(getErrorDerivative());
        if (has_integral)
            output += getFilters()[1].getOutput();
    }
    else if (has_integral)
        output += getFilters()[0].getOutput();
    return output;
}

void PID::configureFirstRun(Time time, const Input &ref, const Input &signal)
{
    FilteredController::configureFirstRun(time, ref, signal);
}

void PID::mapFilterInputs(const Input &ref, const Input &signal, std::vector<linear_system::Input> &input_filters)
{
    Input error = ref - signal;
    linear_system::Input error_integ;
    if (has_integral)
        error_integ = ki.cwiseProduct(error) + gain_antiwidnup.cwiseProduct(getOutput() - getOutputPreSat());
    if (has_derivative)
    {
        input_filters[0] = error;
        if (has_integral)
            input_filters[1] = error_integ;
    }
    else if (has_integral)
        input_filters[0] = error_integ;
}

void PID::mapInitialOutputAndDerivatives(std::vector<Eigen::MatrixXd> &initial_out_dout)
{
    if (has_derivative)
    {
        initial_out_dout[0] = Eigen::MatrixXd::Zero(size(), getFilters()[0].getOrder());
        if (has_integral)
            initial_out_dout[1] = Eigen::MatrixXd::Zero(size(), 1);
    }
    else if (has_integral)
        initial_out_dout[0] = Eigen::MatrixXd::Zero(size(), 1);
}

bool PID::configureFilters(const std::vector<SettingsFilter> & settings)
{
    if (settings.size() > 2)
    {
        std::cerr << "[WARN] (PID::configureFilters) PID must be configured for at most two filters, not "
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
        if (s.getKd() <= 0)
            continue;
        double cutoff = s.getSuggestedDerivativeCutoff();
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
    }
    else
    {
        // place the cutoff frequency two decades ahead
        double damping = 0.8;
        velocity = SettingsFilter::createSecondOrder(damping, max_cutoff);
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
    has_derivative = false;
    for (auto setting : settings)
    {
        kp[k] = setting.getKp();
        ki[k] = setting.getKi();
        kd[k] = setting.getKd();
        weight_reference[k] = setting.weight_reference;
        gain_antiwidnup[k]  = setting.gain_antiwidnup;
        ++k;

        has_integral |= setting.getKi() != 0;
        has_derivative |= setting.getKd() != 0;
    }

    SettingsFilter integrator;
    if (has_integral)
    {
        integrator.num.resize(2);
        integrator.den.resize(2);
        integrator.num << 0, 1;
        integrator.den << 1, 0;
    }
    else
    {
        integrator.num.resize(1);
        integrator.den.resize(1);
        integrator.num << 0;
        integrator.den << 1;
    }
    integrator.sampling_period = settings_velocity_filter.sampling_period;

    if (has_derivative)
    {
        if (has_integral)
            return configureFilters( {settings_velocity_filter, integrator} );
        return configureFilters( {settings_velocity_filter} );
    }
    else if (has_integral)
        return configureFilters( {integrator} );
    return configureFilters({});
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

const Eigen::VectorXd & PID::getErrorDerivative() const
{
    if (!mode_velocity_filtered)
        return dot_error;
    if (has_derivative)
        return getFilters()[0].getOutput();
    return dot_error_zero;
}

void PID::setErrorDerivative(const Input &dot_error)
{
    if (mode_velocity_filtered)
    {
        std::cerr << "[WARN] (PID::updateVelocities) Error velocity is computed inside the PID, so I refuse to directly updated it!" << std::endl;
        return;
    }
    this->dot_error = dot_error;
}
