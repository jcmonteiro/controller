#include "Controller.hpp"
#include <iostream>
#include <limits>


using namespace controller;


Controller::Controller(unsigned int N_controllers) :
    cb_postprocessing(0), cb_saturation(0), _N(N_controllers)
{
    restart();
}

void Controller::restart()
{
    time_last = std::numeric_limits<Time>::max();
    first_run = true;
}

void Controller::setCallbackPostProcessing(Controller::CallbackPostProcessing callback)
{
    cb_postprocessing = callback;
}

void Controller::setCallbackSaturation(Controller::CallbackSaturation callback)
{
    cb_saturation = callback;
}

void Controller::configureFirstRun(Time time, const Input &, const Input &)
{
    time_last = time;
    first_run = false;
    output = getDefaultOutput();
    output_presat = output;
}

void Controller::update(Time time, const Input &ref, const Input &signal)
{
    if (ref.size() != _N || signal.size() != _N)
    {
        std::stringstream error;
        error << "[ERROR] (Controller::update) the size of at least one argument != "
              << _N << ", which is the controller dimension";
        throw std::logic_error(error.str());
    }
    if (time <= time_last)
    {
        if (first_run)
            configureFirstRun(time, ref, signal);
        else
        {
            if (time < time_last)
                std::cerr << "[WARN] (Controller::update) update called for a past time; output will not change" << std::endl;
            else
                std::cerr << "[WARN] (Controller::update) update called but no time has passed since the last call; output will not change" << std::endl;
            return;
        }
    }
    output = updateControl(time, ref, signal);
    if (cb_postprocessing)
        cb_postprocessing(output);
    output_presat = output;
    if (cb_saturation)
        cb_saturation(output);
    time_last = time;
}
