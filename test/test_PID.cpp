#define BOOST_TEST_MODULE LinearSystem test

#include <boost/test/unit_test.hpp>
#include <iostream>
#include "PID.hpp"

using namespace pid_control;

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

BOOST_AUTO_TEST_CASE(test_class_methods)
{
    std::cout << "[TEST] class basic methods" << std::endl;
    PID pid;
    //
    ParallelPIDSettings settings;
    settings.B = 1;
    settings.Ki = 5;
    settings.Kd = 0.2;
    settings.Kp = 19;
    settings.Tt = 1 / settings.Ki;
    settings.Ts = 1;
    //
    PID::Parameters params;
    params.pid_settings.push_back(settings);
    params.integration_method = linear_system::TUSTIN;
    params.saturate = true;
    params.wrap_2pi = false;
    //
    pid.configParameters(params);

    // run twice and see if we get the same results
    // this should verify that restart is working properly
    PID::Inputs input;
    PID::Outputs output;
    double time = 0, dt = 1;
    unsigned int n_turns = 10;
    std::vector<double> history(n_turns);
    pid.restart();
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        input = PID::Inputs::create(1.0, 0.2, 0.2, 0.3, 0);
        input.time = linear_system::LinearSystem::getTimeFromSeconds( time += dt );
        output = pid.update(input);
        history[k] = output.pid[0];
    }
    pid.restart();
    time = 0;
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        input = PID::Inputs::create(1.0, 0.2, 0.2, 0.3, 0);
        input.time = linear_system::LinearSystem::getTimeFromSeconds( time += dt );
        output = pid.update(input);
        if (std::abs(history[k] - output.pid[0]) != 0)
            BOOST_ERROR("found different output values when running the same PID twice");
    }
}
