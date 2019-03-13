#pragma once


#include "Controller.hpp"
#include <linear_system/LinearSystem.hpp>


namespace controller
{


struct SettingsFilters
{
    Eigen::VectorXd num, den;
    Eigen::MatrixXd init_output_and_derivs;
    double prewarp;
    double sampling_period;
    unsigned int n_filters;
    linear_system::IntegrationMethod method;
};


class FilteredController : public Controller
{
private:
    unsigned int N_filters;

    std::vector<linear_system::LinearSystem> filters;

protected:
    void updateFilters(Time time, const std::vector<Input> & inputs);

public:
    explicit inline FilteredController(int N_controllers, unsigned int N_filters = 1) :
        Controller(N_controllers),
        N_filters(N_filters)
    {
        filters.resize(N_filters);
    }

    virtual inline bool ok() const
    {
        return filters.size() == N_filters;
    }

    bool configureFilters(const std::vector<SettingsFilters> & settings);
};


}
