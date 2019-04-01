#include "FilteredController.hpp"
#include <linear_system/HelperFunctions.hpp>
#include <iostream>


using namespace controller;


SettingsFilter::SettingsFilter() :
    num(), den(),
    prewarp(0), sampling_period(0), method(linear_system::TUSTIN)
{
}

SettingsFilter SettingsFilter::createSecondOrder(double damp, double cutoff)
{
    SettingsFilter ret;
    double wn = linear_system::cutoff2resonant(cutoff, damp);
    ret.num.resize(3);
    ret.den.resize(3);
    ret.num << 0, wn*wn, 0;
    ret.den << 1, 2*damp*wn, wn*wn;
    return ret;
}

FilteredController::FilteredController(unsigned int N_controllers, unsigned int N_filters) :
    Controller(N_controllers),
    N_filters(N_filters),
    filters(N_filters),
    filters_inputs(N_filters)
{
}

void FilteredController::updateFilters(Time time, const std::vector<linear_system::Input> & inputs)
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
        throw std::logic_error("[ERROR] (FilteredController::_mapFilterInputs) "
                               "<child class>::mapFilterInputs changed the filters "
                               "inputs size when it shouldn't have!");
}

void FilteredController::_mapInitialOutputAndDerivatives()
{
    mapInitialOutputAndDerivatives(filters_init_out_dout);
    if (filters_init_out_dout.size() != N_filters)
        throw std::logic_error("[ERROR] (FilteredController::_mapInitialOutputAndDerivatives) "
                               "<child class>::mapInitialOutputAndDerivatives changed the filters "
                               "inputs size when it shouldn't have!");
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
    _mapInitialOutputAndDerivatives();
    auto iter_filters  = filters.begin();
    auto iter_inputs   = filters_inputs.cbegin();
    auto iter_out_dout = filters_init_out_dout.cbegin();
    while (iter_filters != filters.end())
    {
        iter_filters->setInitialTime(time);
        iter_filters->setInitialConditions(
            iter_inputs->transpose().replicate(1, iter_filters->getOrder()),
            *iter_out_dout
        );
        ++iter_filters;
        ++iter_inputs;
        ++iter_out_dout;
    }
}

bool FilteredController::configureFilters(const std::vector<SettingsFilter> & settings)
{
    N_filters = settings.size();
    filters.resize(N_filters);
    filters_inputs.resize(N_filters);
    filters_init_out_dout.resize(N_filters);

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
        *iter_filters = linear_system::LinearSystem(iter_settings->num, iter_settings->den,
            iter_settings->sampling_period, iter_settings->method, iter_settings->prewarp);
        iter_filters->setMaximumTimeBetweenUpdates(10 * max_sampling);
        iter_filters->useNFilters(_N);
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
