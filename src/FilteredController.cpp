#include "FilteredController.hpp"
#include <iostream>


using namespace controller;


void FilteredController::updateFilters(Time time, const std::vector<Controller::Input> & inputs)
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

bool FilteredController::configureFilters(const std::vector<SettingsFilters> & settings)
{
    N_filters = settings.size();
    filters.resize(N_filters);

    if (settings.empty())
    {
        std::cerr << "[WARN] (FilteredController::setNFilters) I have no filters! Is this intended?" << std::endl;
        return ok();
    }

    auto iter_settings = settings.cbegin();
    auto iter_filters  = filters.begin();
    while (iter_settings != settings.cend())
    {
        iter_filters->setIntegrationMethod(iter_settings->method);
        iter_filters->setSampling(iter_settings->sampling_period);
        iter_filters->setFilter(iter_settings->num, iter_settings->den);
        iter_filters->useNFilters(iter_settings->n_filters);
        iter_filters->setInitialOutputDerivatives(iter_settings->init_output_and_derivs);
        ++iter_settings;
        ++iter_filters;
    }

    if (!ok())
    {
        std::cerr << "[WARN] (FilteredController::setNFilters) Failed to configure filters!" << std::endl;
        N_filters = 0;
        filters.resize(0);
        return false;
    }
    return true;
}
