#include "Controller.hpp"
#include <iostream>
#include <limits>


using namespace controller;


Controller::Controller(unsigned int N_controllers) :
    _N(N_controllers)
{
    restart();
}

void Controller::restart()
{
    time_last = std::numeric_limits<Time>::max();
    first_run = true;
}

void Controller::configureFirstRun(Time time, const Input &, const Input &)
{
    time_last = time;
    first_run = false;
}

void Controller::update(Time time, const Input &ref, const Input &signal, const Output &last_output)
{
    if (time <= time_last)
    {
        if (first_run)
            configureFirstRun(time, ref, signal);
        else
        {
            std::cerr << "[WARN] (Controller::update) update called for a past time; output set to its default value" << std::endl;
            output = getDefaultOutput();
            return;
        }
    }
    output = updateControl(time, ref, signal, last_output);
    time_last = time;
}
