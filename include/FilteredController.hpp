#pragma once


#include "Controller.hpp"
#include <linear_system/LinearSystem.hpp>


namespace controller
{


struct SettingsFilter
{
    Eigen::VectorXd num, den;
    double prewarp;
    double sampling_period;
    linear_system::IntegrationMethod method;

    SettingsFilter();

    static SettingsFilter createSecondOrder(double damp, double cutoff);
};


class FilteredController : public Controller
{
private:
    unsigned int N_filters;

    std::vector<linear_system::LinearSystem> filters;
    std::vector<linear_system::Input> filters_inputs;
    std::vector<Eigen::MatrixXd> filters_init_out_dout;

    void updateFilters(Time time, const std::vector<linear_system::Input> &inputs);

    /**
     * @brief Calls the child implementation of #mapFilterInputs
     * with \p filters_inpupts = FilteredController#filters_inputs and
     * throws std::logic_error if the size of \p filters_inputs is changed.
     * @throws std::logic_error
     */
    void _mapFilterInputs(const Input &ref, const Input &signal);

    /**
     * @brief Calls the child implementation of #mapInitialOutputAndDerivatives
     * with \p init_out_dout = FilteredController#filters_init_out_dout and
     * throws std::logic_error if the size of \p filters_init_out_dout is changed.
     * @throws std::logic_error
     */
    void _mapInitialOutputAndDerivatives();

protected:
    /**
     * @brief Updates the filter-side of the control algorithm
     *
     * @throws std::logic_error If the child class implementation of #mapFilterInputs
     * changes the size of its \p input_filters parameter.
     */
    virtual const Output & updateControl(Time time, const Input &ref, const Input &signal);

    /**
     * @brief Called from Controller#update during the first call after Controller#reset.
     */
    virtual void configureFirstRun(Time time, const Input &ref, const Input &signal);

    /**
     * @brief Maps the received inputs onto the filters input vector
     *
     * You should never change the size of \p input_filters!
     *
     * @param input_filters Vector that will have its elements modified to hold the filters inputs.
     */
    virtual void mapFilterInputs(const Input &ref, const Input &signal, std::vector<linear_system::Input> &input_filters) = 0;

    /**
     * @brief Returns the initial output and output derivatives.
     * @param init_out_dout A (LinearSystem#getNFilters by LinearSystem#getOrder) matrix where each row
     * contains y_i[0], dy_i/dt[0], ..., d^(N-1)y_i/dt^(N-1)[0], the i-th output and its N-1 derivatives.
     */
    virtual void mapInitialOutputAndDerivatives(std::vector<Eigen::MatrixXd> &init_out_dout) = 0;

    /**
     * @brief Configures each filter using the provided SettingsFilter \p settings vector.
     *
     * Each time this method is called it discards the last set of filters and creates a new one.
     *
     * @param settings Each element is used to create and configure a new filter.
     * @returns True to indicate that the filters were successfuly configures. False otherwise.
     */
    virtual bool configureFilters(const std::vector<SettingsFilter> & settings);

    /**
     * @brief Returns a constant reference to the filters vector.
     */
    inline const std::vector<linear_system::LinearSystem> & getFilters() const
    {
        return filters;
    }

public:
    explicit FilteredController(unsigned int N_controllers, unsigned int N_filters = 1);

    virtual inline bool ok() const
    {
        return filters.size() == N_filters && filters_inputs.size() == N_filters;
    }
};


}
