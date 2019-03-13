#include <sstream>
#include "PID.hpp"
#include <linear_system/HelperFunctions.hpp>


using namespace controller;


void PID::Inputs::checkDimensions(unsigned int dim) const
{
    if (dim != reference.size() || dim != signal.size() || dim != dotreference.size() || dim != dotsignal.size() || dim != pid_sat.size())
    {
        std::stringstream error;
        error << "(PID) wrong dimension on inputs" << std::endl
            << " expected dim: " << dim << std::endl
            << " reference: " << reference.size() << std::endl
            << " signal: " << signal.size() << std::endl
            << " dotreference: " << dotreference.size() << std::endl
            << " dotsignal: " << dotsignal.size() << std::endl
            << " pid_sat: " << pid_sat.size();
        throw std::runtime_error(error.str());
    }
}

void PID::Inputs::resize(unsigned int dim)
{
    reference.resize(dim);
    signal.resize(dim);
    dotreference.resize(dim);
    dotsignal.resize(dim);
    pid_sat.resize(dim);
}

PID::Inputs PID::Inputs::create(double ref, double sig, double dref, double dsig, double sat)
{
    Inputs ret;
    ret.reference.setConstant(1, ref);
    ret.signal.setConstant(1, sig);
    ret.dotreference.setConstant(1, dref);
    ret.dotsignal.setConstant(1, dsig);
    ret.pid_sat.setConstant(1, sat);
    return ret;
}

void PID::Outputs::resize(unsigned int dim)
{
    p_error.setZero(dim);
    d_error.setZero(dim);
    p.setZero(dim);
    i.setZero(dim);
    d.setZero(dim);
    pid.setZero(dim);
}


void PID::configIntegrator()
{
    // assume all checks have been done on the parameters by now
    integrator.setIntegrationMethod(params.integration_method);
    integrator.setSampling(params.pid_settings[0].Ts);
    integrator.useNFilters(dim);
    Eigen::VectorXd tfNum(2), tfDen(2);
    tfNum << 0,1;
    tfDen << 1,0;
    integrator.setFilter(tfNum, tfDen);
    integrator.setInitialOutputDerivatives(Eigen::MatrixXd::Zero(dim,1));
    integrator.discretizeSystem();
}

void PID::configParameters(const Parameters &params)
{
    dim = params.pid_settings.size();
    if (dim <= 0)
        throw std::logic_error("pid settings size must be bigger than 0");

    this->params = params;
    outputs.resize(dim);
    p_error_with_B.resize(dim);
    integrator_input.resize(dim);
    B.resize(dim);
    Kp.resize(dim);
    Ki.resize(dim);
    Kd.resize(dim);
    Kt.resize(dim);

    for (unsigned int i = 0; i < dim; i++)
    {
        B[i] = params.pid_settings[i].B;
        Kp[i] = params.pid_settings[i].Kp;
        Ki[i] = params.pid_settings[i].Ki;
        Kd[i] = params.pid_settings[i].Kd;

        if (params.pid_settings[i].Tt > 0)
            Kt[i] = 1/params.pid_settings[i].Tt;
        else if ((params.pid_settings[i].Tt == 0) && (params.pid_settings[i].Kd > 0))
            Kt[i] = sqrt(params.pid_settings[i].Ki/params.pid_settings[i].Kd);
        else
            Kt[i] = 0;

        if (params.pid_settings[i].Ts != params.pid_settings[0].Ts)
            throw std::logic_error("All pid settings must have the same Ts");
    }
    has_integrator = (Ki.array() != 0).any();
    if (has_integrator)
        configIntegrator();
}

const PID::Outputs & PID::update(const Inputs & inputs, bool check_dim)
{
    if (check_dim)
        inputs.checkDimensions(dim);

    // calculate errors
    outputs.p_error = inputs.reference - inputs.signal;
    p_error_with_B = B.cwiseProduct(inputs.reference) - inputs.signal;
    outputs.d_error = inputs.dotreference - inputs.dotsignal;
    if (params.wrap_2pi)
    {
        linear_system::wrap2pi(outputs.p_error);
        linear_system::wrap2pi(p_error_with_B);
    }

    // saturation
    if (first_run && params.saturate)
        outputs.pid = inputs.pid_sat;

    // integrator
    integrator_input = Ki.cwiseProduct(outputs.p_error);
    if (params.saturate && has_integrator)
        integrator_input += Kt.cwiseProduct(inputs.pid_sat - outputs.pid);
    if (first_run)
    {
        if (has_integrator)
        {
            integrator.setInitialState(integrator_input);
            integrator.setInitialTime(inputs.time);
        }
        first_run = false;
    }

    // calculate pid
    outputs.p = Kp.cwiseProduct(p_error_with_B);
    if (has_integrator)
        outputs.i = integrator.update(integrator_input, inputs.time);
    outputs.d = Kd.cwiseProduct(outputs.d_error);
    outputs.pid = outputs.p + outputs.i + outputs.d;

    return outputs;
}
