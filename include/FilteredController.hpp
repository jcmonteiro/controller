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
    linear_system::IntegrationMethod method;

    SettingsFilters();
};


class FilteredController : public Controller
{
private:
    unsigned int N_filters;

    std::vector<linear_system::LinearSystem> filters;
    std::vector<Input> filters_inputs;

    void updateFilters(Time time, const std::vector<Input> &inputs);

    /**
     * @brief _mapFilterInputs Calls the child implementation of #mapFilterInputs
     * with \p filters_inpupts = FilteredController#filters_inputs and
     * throws std::logic_error if the size of input_filters is changed.
     * @throws std::logic_error
     */
    void _mapFilterInputs(const Input &ref, const Input &signal);

protected:
    /**
     * @brief updateControl Updates the filter-side of the control algorithm
     *
     * @throws std::logic_error If the child class implementation of #mapFilterInputs
     * changes the size of its \p input_filters parameter.
     */
    virtual const Output & updateControl(Time time, const Input &ref, const Input &signal, const Output &last_output);

    virtual void configureFirstRun(Time time, const Input &ref, const Input &signal);

    /**
     * @brief mapFilterInputs Map the received inputs onto the filters input vector
     *
     * You should never change the size of \p input_filters!
     *
     * @param input_filters Vector that will have its elements modified to hold the filters inputs.
     */
    virtual void mapFilterInputs(const Input &ref, const Input &signal, std::vector<Input> &input_filters) = 0;

    bool configureFilters(const std::vector<SettingsFilters> & settings);

public:
    explicit inline FilteredController(unsigned int N_controllers, unsigned int N_filters = 1) :
        Controller(N_controllers),
        N_filters(N_filters)
    {
        filters.resize(N_filters);
        filters_inputs.resize(N_filters);
    }

    virtual inline bool ok() const
    {
        return filters.size() == N_filters && filters_inputs.size() == N_filters;
    }
};


}
