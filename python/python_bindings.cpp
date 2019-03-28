#include <pybind11/pybind11.h>

// needed for Eigen matrices
#include <pybind11/eigen.h>

// needed for vectors
#include <pybind11/stl.h>

#include "PID.hpp"

namespace py = pybind11;
using namespace controller;

bool configureAdvanced(PID &pid, const std::vector<SettingsPID> & settings, const SettingsFilter &settings_velocity_filter)
{
    return pid.configure(settings, settings_velocity_filter);
}
bool configureBasic(PID &pid, const SettingsPID &settings, double sampling)
{
    return pid.configure(settings, sampling);
}

PYBIND11_MODULE(controller_py, m) {
    py::class_<SettingsPID>(m, "SettingsPID")
        .def(py::init<double, double, double>())
    ;

    py::class_<PID>(m, "PID")
        .def(py::init<unsigned int>())
        .def("clearCallbackPostProcessing", &PID::clearCallbackPostProcessing)
        .def("clearCallbackSaturation",     &PID::clearCallbackSaturation)
        .def("configureAdvanced",           &configureAdvanced)
        .def("configureBasic",              &configureBasic)
        .def("getDefaultOutput",            &PID::getDefaultOutput)
        .def("getOutput",                   &PID::getOutput)
        .def("getOutputPreSat",             &PID::getOutputPreSat)
        .def("ok",                          &PID::ok)
        .def("restart",                     &PID::restart)
        .def("setCallbackPostProcessing",   &PID::setCallbackPostProcessing)
        .def("setCallbackSaturation",       &PID::setCallbackSaturation)
        .def("size",                        &PID::size)
        .def("update",                      &PID::update)
        .def("updateVelocities",            &PID::setErrorDerivative)
    ;
}
