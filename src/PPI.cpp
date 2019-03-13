#include "PPI.hpp"


using namespace controller;


void PPI::Inputs::checkDimensions(unsigned int dim) const
{
    if (dim != reference.size() || dim != signal.size() || dim != dotreference.size() || dim != dotsignal.size() || dim != p_pi_sat.size())
    {
        std::stringstream error;
        error << "(PPI) wrong dimension on inputs" << std::endl
            << " expected dim: " << dim << std::endl
            << " reference: " << reference.size() << std::endl
            << " signal: " << signal.size() << std::endl
            << " dotreference: " << dotreference.size() << std::endl
            << " dotsignal: " << dotsignal.size() << std::endl
            << " p_pi_sat: " << p_pi_sat.size();
        throw std::runtime_error(error.str());
    }
}

void PPI::Inputs::resize(unsigned int dim)
{
    reference.resize(dim);
    signal.resize(dim);
    dotreference.resize(dim);
    dotsignal.resize(dim);
}

PPI::Inputs PPI::Inputs::create(double ref, double sig, double dref, double dsig, double sat)
{
    Inputs ret;
    ret.reference.setConstant(1, ref);
    ret.signal.setConstant(1, sig);
    ret.dotreference.setConstant(1, dref);
    ret.dotsignal.setConstant(1, dsig);
    ret.p_pi_sat.setConstant(1, sat);
    return ret;
}

void PPI::Outputs::resize(unsigned int dim)
{
    p_error.resize(dim);
    d_error.resize(dim);
    p_pi.resize(dim);
}

void PPI::configParameters(const Parameters &params)
{
    dim = params.p_pi_settings.size();
    if (dim <= 0)
        throw std::logic_error("pid settings size must be bigger than 0");

    this->params = params;
    outputs.resize(dim);
    pid_inputs.resize(dim);

    pid_params.pid_settings.resize(dim);
    for (unsigned int i = 0; i < dim; i++)
    {
        if (params.p_pi_settings[i].Ts != params.p_pi_settings[0].Ts)
            throw std::logic_error("All P-PI settings must have the same Ts");

        pid_params.pid_settings[i].Ts = params.p_pi_settings[i].Ts;
        pid_params.pid_settings[i].Kp = params.p_pi_settings[i].Kp1*params.p_pi_settings[i].Kp2 + params.p_pi_settings[i].Ki;
        pid_params.pid_settings[i].Ki = params.p_pi_settings[i].Kp1*params.p_pi_settings[i].Ki;
        pid_params.pid_settings[i].Kd = params.p_pi_settings[i].Kp2;
        pid_params.pid_settings[i].B = params.p_pi_settings[i].B;
        pid_params.pid_settings[i].Tt = params.p_pi_settings[i].Tt;
        pid_params.pid_settings[i].YMin = params.p_pi_settings[i].YMin;
        pid_params.pid_settings[i].YMax = params.p_pi_settings[i].YMax;
    }

    pid_params.integration_method = params.integration_method;
    pid_params.wrap_2pi = params.wrap_2pi;
    pid_params.saturate = params.saturate;

    pid.configParameters(pid_params);
}

const PPI::Outputs & PPI::update(const Inputs &inputs, bool check_dim)
{
    if (check_dim)
        inputs.checkDimensions(dim);

    pid_inputs.reference = inputs.reference;
    pid_inputs.dotreference = inputs.dotreference;
    pid_inputs.signal = inputs.signal;
    pid_inputs.dotsignal = inputs.dotsignal;
    pid_inputs.pid_sat = inputs.p_pi_sat;
    pid_inputs.time = inputs.time;
    const PID::Outputs & pid_outputs = pid.update(pid_inputs, false);
    outputs.p_error = pid_outputs.p_error;
    outputs.d_error = pid_outputs.d_error;
    outputs.p_pi = pid_outputs.pid;
    return outputs;
}
