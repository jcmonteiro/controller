#define BOOST_TEST_MODULE LinearSystem test

#include <boost/test/unit_test.hpp>
#include <iostream>
#include "PIDController.hpp"

using namespace controller;

void printProgress(int width, float progress)
{
    std::cout << "[";
    int pos = width * progress;
    for (int i = 0; i < width; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
    if (progress == 1.0)
        std::cout << std::endl;
}

void testReset(Controller &controller)
{
    // run twice and see if we get the same results
    // this should verify that restart is working properly
    const unsigned int N = controller.size();
    Input ref(N), sig(N);
    ref << 1, 0.8;
    ref << 0.5, 0.6;
    Output out = controller.getDefaultOutput();
    double time = 0, dt = 1;
    unsigned int n_turns = 10;
    std::vector<Output> history(n_turns);
    controller.restart();
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds( time += dt ),
            ref,
            sig,
            out
        );
        history[k] = controller.getOutput();
    }
    controller.restart();
    out = controller.getDefaultOutput();
    time = 0;
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds( time += dt ),
            ref,
            sig,
            out
        );
        out = controller.getOutput();
        if ( ((out - history[k]).array() != 0).any() )
            BOOST_ERROR("found different output values when running the same controller twice");
    }
}

void testInputChannels(Controller &controller)
{
    const unsigned int N = controller.size();
    Input ref(N), sig(N);
    ref.setConstant(23.43);
    sig.setConstant(11.33);
    Output out = controller.getDefaultOutput();
    double time = 0, dt = 1;
    unsigned int n_turns = 10;
    controller.restart();
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds( time += dt ),
            ref,
            sig,
            out
        );
        out = controller.getOutput();
        if ( ((out.array() - out[0]) != 0).any() )
            BOOST_ERROR("found different output values while sending the same inputs to all controller channels");
    }
}

void testStepResponse(Controller &controller, const SettingsPID &settings, double sampling, double time_settling, double max_overshoot, double tol_settling_error)
{
    Input ref(1);
    ref.setConstant(1);

    linear_system::LinearSystem plant;
    Eigen::VectorXd num(3), den(3);
    num << 0, 0, 1;
    den << 1, 0, 0;
    plant.setFilter(num, den);
    plant.setSampling(sampling);
    Eigen::MatrixXd init_out(1, 2);
    init_out << 0, 0;
    plant.setInitialOutputDerivatives(init_out);
    plant.discretizeSystem();

    linear_system::LinearSystem prefilter;
    Eigen::VectorXd num_filter(3), den_filter(3);
    num_filter << 0, 0, settings.ki;
    den_filter << settings.kd, settings.kp, settings.ki;
    prefilter.setFilter(num_filter, den_filter);
    prefilter.setSampling(sampling);
    Eigen::MatrixXd init_out_filter(1, 2);
    init_out_filter << plant.getOutput()[0], 0;
    prefilter.setInitialOutputDerivatives(init_out_filter);
    prefilter.discretizeSystem();

    Eigen::MatrixXd init_in(1, 2), init_in_filter(1, 2);
    init_in.setZero();
    init_in_filter << ref[0], ref[0];
    plant.setInitialState(init_in);
    prefilter.setInitialState(init_in_filter);

    double time = 0, dt = sampling;
    double ratio = 0;
    double overshoot = 0;
    double time_overshoot = 0;
    plant.setInitialTime(time);
    prefilter.setInitialTime(time);
    controller.restart();
    while (time < time_settling)
    {
        prefilter.update(
            ref,
            linear_system::LinearSystem::getTimeFromSeconds(time)
        );
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds(time),
            prefilter.getOutput(),
            plant.getOutput(),
            plant.getOutput()
        );
        plant.update(controller.getOutput(), linear_system::LinearSystem::getTimeFromSeconds(time));
        ratio = std::abs( plant.getOutput()[0]/ref[0] - 1 );
        if (plant.getOutput()[0] > ref[0] && ratio > overshoot)
        {
            overshoot = ratio;
            time_overshoot = time;
        }
        time += dt;
    }
    if (overshoot > max_overshoot)
    {
        std::stringstream error;
        error << "PID exceeded the maximum overshoot at t = " << time_overshoot/time_settling
              << " ts : (max_overshoot, overshoot) = ("
              << max_overshoot << " , " << overshoot << ")";
        BOOST_ERROR(error.str());
    }
    if (ratio >= tol_settling_error)
    {
        std::stringstream error;
        error << "PID did not meet the required final performance: "
              << "(ref , out) = (" << 1 << " , " << plant.getOutput()[0]/ref[0]
              << ")";
        BOOST_ERROR(error.str());
    }
}

BOOST_AUTO_TEST_CASE(test_channels)
{
    std::cout << "[TEST] same input to multiple channels" << std::endl;
    //
    PID pid(2);

    SettingsPID settings;
    settings.kp = 1;
    settings.kd = 0.2;
    settings.ki = 2;

    pid.configure({settings, settings}, 0.2);
    //
    testReset(pid);
}

BOOST_AUTO_TEST_CASE(test_reset)
{
    std::cout << "[TEST] reset" << std::endl;
    //
    unsigned int N = 20;
    PID pid(N);

    SettingsPID settings;
    settings.kp = 1;
    settings.kd = 0.2;
    settings.ki = 2;

    std::vector<SettingsPID> settings_all(N);
    for (auto &s : settings_all)
        s = settings;

    pid.configure(settings_all, 0.2);
    //
    testInputChannels(pid);
}

BOOST_AUTO_TEST_CASE(test_step_response)
{
    std::cout << "[TEST] step response" << std::endl;
    //
    PID pid(1);
    double ts_array[] = {0.1, 1, 10, 100, 1000};
    double overshoot_array[] = {0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5};
    double tol_ratio_overshoot = 1.1;
    double tol_settling_error = 0.025;
    for (double over : overshoot_array)
    {
        for (double ts : ts_array)
        {
            auto settings = SettingsPID::createFromSpecT(over, ts);
            double sampling = settings.getSuggestedSampling();
            pid.configure(settings, sampling);
            //
            testStepResponse(pid, settings, sampling, ts, tol_ratio_overshoot * over, tol_settling_error);
        }
    }
}
