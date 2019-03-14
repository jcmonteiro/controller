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
    // run twice and see if we get the same results
    // this should verify that restart is working properly
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
