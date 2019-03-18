#include "FilteredController.hpp"
#include <iostream>


using namespace controller;


SettingsFilter::SettingsFilter() :
    num(), den(), init_output_and_derivs(),
    prewarp(0), sampling_period(0), method(linear_system::TUSTIN)
{
}


SettingsFilter SettingsFilter::createSecondOrder(double damp, double cutoff, unsigned int N)
{
    SettingsFilter ret;
    double wn = cutoff / damp;
    ret.num.resize(3);
    ret.den.resize(3);
    ret.num << 0, wn*wn, 0;
    ret.den << 1, 2*damp*wn, wn*wn;
    ret.init_output_and_derivs.setZero(N, 2);
    return ret;
}

void FilteredController::updateFilters(Time time, const std::vector<Input> & inputs)
{
    if (inputs.size() != filters.size())
    {
        if (inputs.size() > filters.size())
            throw std::logic_error("[ERROR] (FilteredController::updateFilters) there must be as many filters as inputs");
        else
            throw std::logic_error("[ERROR] (FilteredController::updateFilters) there must be as many inputs as filters");
    }
    auto iter_input = inputs.cbegin();
    auto iter_filters = filters.begin();
    while (iter_input != inputs.cend())
    {
        iter_filters->update(*iter_input, time);
        ++iter_input;
        ++iter_filters;
    }
}

void FilteredController::_mapFilterInputs(const Input &ref, const Input &signal)
{
    mapFilterInputs(ref, signal, filters_inputs);
    if (filters_inputs.size() != N_filters)
        throw std::logic_error("[ERROR] (FilteredController::updateControl) <child class>::mapFilterInputs changed the filters inputs size when it shouldn't have!");
}

const Output & FilteredController::updateControl(Time time, const Input &ref, const Input &signal)
{
    _mapFilterInputs(ref, signal);
    updateFilters(time, filters_inputs);
    return getOutput();
}

void FilteredController::configureFirstRun(Time time, const Input &ref, const Input &signal)
{
    Controller::configureFirstRun(time, ref, signal);
    _mapFilterInputs(ref, signal);
    auto iter_filters = filters.begin();
    auto iter_inputs  = filters_inputs.cbegin();
    while (iter_filters != filters.end())
    {
        iter_filters->setInitialTime(time);
        iter_filters->setInitialState(
            static_cast<Eigen::MatrixXd>(
                iter_inputs->replicate(1, iter_filters->getOrder())
            )
        );
        ++iter_filters;
        ++iter_inputs;
    }
}

bool FilteredController::configureFilters(const std::vector<SettingsFilter> & settings)
{
    N_filters = settings.size();
    filters.resize(N_filters);
    filters_inputs.resize(N_filters);

    if (settings.empty())
    {
        std::cerr << "[WARN] (FilteredController::configureFilters) I have no filters! Is this intended?" << std::endl;
        return ok();
    }

    double max_sampling = 0;
    for (auto s : settings)
    {
        if (s.sampling_period > max_sampling)
            max_sampling = s.sampling_period;
    }

    auto iter_settings = settings.cbegin();
    auto iter_filters  = filters.begin();
    auto iter_inputs   = filters_inputs.begin();
    while (iter_settings != settings.cend())
    {
        iter_filters->setIntegrationMethod(iter_settings->method);
        iter_filters->setSampling(iter_settings->sampling_period);
        iter_filters->setMaximumTimeBetweenUpdates(10 * max_sampling);
        iter_filters->setFilter(iter_settings->num, iter_settings->den);
        iter_filters->useNFilters(_N);
        iter_filters->setInitialOutputDerivatives(iter_settings->init_output_and_derivs);
        iter_filters->discretizeSystem();
        iter_inputs->setZero(_N);
        ++iter_settings;
        ++iter_filters;
        ++iter_inputs;
    }

    if (!ok())
    {
        std::cerr << "[WARN] (FilteredController::configureFilters) Failed to configure filters!" << std::endl;
        N_filters = 0;
        filters.resize(0);
        filters_inputs.resize(0);
        return false;
    }
    return true;
}
