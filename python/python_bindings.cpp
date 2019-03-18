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
        .def(py::init<>())
        .def_static("createFromSpecT", SettingsPID::createFromSpecT)
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
        .def("updateVelocities",            &PID::updateVelocities)
    ;

//    py::class_<SettingsPID>(m, "SettingsPID")
//        // constructor
//        .def(py::init<>())
//        // methods
//        .def("__eq__", &PID::ParallelSettings::operator==)
//        .def("__ne__", &PID::ParallelSettings::operator!=)
//        // public member variables
//        .def_readwrite("Ts",   &PID::ParallelSettings::Ts)
//        .def_readwrite("Kp",   &PID::ParallelSettings::Kp)
//        .def_readwrite("Ki",   &PID::ParallelSettings::Ki)
//        .def_readwrite("Kd",   &PID::ParallelSettings::Kd)
//        .def_readwrite("B",    &PID::ParallelSettings::B)
//        .def_readwrite("Tt",   &PID::ParallelSettings::Tt)
//        .def_readwrite("YMin", &PID::ParallelSettings::YMin)
//        .def_readwrite("YMax", &PID::ParallelSettings::YMax)
//    ;

//    py::class_<PID::Parameters>(m, "PIDParameters")
//        // constructor
//        .def(py::init<>())
//        // methods
//        .def("setSettings", &PID::Parameters::setSettings)
//        // public member variables
//        .def_readwrite("integration_method", &PID::Parameters::integration_method)
//        .def_readwrite("wrap", &PID::Parameters::wrap_2pi)
//        .def_readwrite("saturate", &PID::Parameters::saturate)
//    ;

//    py::class_<PID::Inputs>(m, "PIDInput")
//        // constructor
//        .def(py::init<>())
//        // methods
//        .def("checkDimensions", &PID::Inputs::checkDimensions)
//        .def("resize", &PID::Inputs::resize)
//        // static methods
//        .def_static("create", &PID::Inputs::create)
//        // public member variables
//        .def_readwrite("time", &PID::Inputs::time)
//        .def_readwrite("ref", &PID::Inputs::reference)
//        .def_readwrite("sig", &PID::Inputs::signal)
//        .def_readwrite("dref", &PID::Inputs::dotreference)
//        .def_readwrite("dsig", &PID::Inputs::dotsignal)
//        .def_readwrite("sat", &PID::Inputs::pid_sat)
//    ;

//    py::class_<PID::Outputs>(m, "PIDOutput")
//        // constructor
//        .def(py::init<>())
//        // methods
//        .def("resize", &PID::Outputs::resize)
//        .def("getValue", &PID::Outputs::getValue)
//    ;
}
