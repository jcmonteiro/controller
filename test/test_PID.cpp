#define BOOST_TEST_MODULE LinearSystem test

#include <boost/test/unit_test.hpp>
#include <iostream>
#include "PID.hpp"
#include "PPI.hpp"

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

template <class Controller>
void testControllerClassMethods(const typename Controller::ParallelSettings &settings)
{
    typedef typename Controller::Inputs Input;
    typedef typename Controller::Outputs Output;
    typedef typename Controller::Parameters Parameters;
    typedef typename Controller::ParallelSettings Settings;

    Parameters params;
    std::vector<Settings> vec_settings;
    vec_settings.push_back(settings);
    params.setSettings(vec_settings);
    params.integration_method = linear_system::TUSTIN;
    params.saturate = true;
    params.wrap_2pi = false;
    //
    Controller controller;
    controller.configParameters(params);

    // run twice and see if we get the same results
    // this should verify that restart is working properly
    auto input = Input::create(1.0, 0.9, 0.2, 0.3, 0);
    Output output;
    double time = 0, dt = 1;
    unsigned int n_turns = 10;
    std::vector<double> history(n_turns);
    controller.restart();
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        input.time = linear_system::LinearSystem::getTimeFromSeconds( time += dt );
        output = controller.update(input);
        history[k] = output.getValue()[0];
    }
    controller.restart();
    time = 0;
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        input.time = linear_system::LinearSystem::getTimeFromSeconds( time += dt );
        output = controller.update(input);
        if (std::abs(history[k] - output.getValue()[0]) != 0)
            BOOST_ERROR("found different output values when running the same controller twice");
    }
}

BOOST_AUTO_TEST_CASE(test_class_methods)
{
    std::cout << "[TEST] class basic methods" << std::endl;
    //
    PID::ParallelSettings pid_settings;
    pid_settings.B = 1;
    pid_settings.Ki = 5;
    pid_settings.Kd = 0.2;
    pid_settings.Kp = 19;
    pid_settings.Tt = 1 / pid_settings.Ki;
    pid_settings.Ts = 1;
    pid_settings.YMin = -10;
    pid_settings.YMax = 10;
    //
    PPI::ParallelSettings ppi_settings;
    ppi_settings.B = 1;
    ppi_settings.Ki = 16;
    ppi_settings.Kp2 = 8;
    ppi_settings.Kp1 = 2;
    ppi_settings.Tt = 1 / ppi_settings.Ki;
    ppi_settings.Ts = 1;
    ppi_settings.YMin = -10;
    ppi_settings.YMax = 10;
    //
    testControllerClassMethods<PID>(pid_settings);
    testControllerClassMethods<PPI>(ppi_settings);
}
