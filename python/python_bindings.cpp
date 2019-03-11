#include <pybind11/pybind11.h>

// needed for Eigen matrices
#include <pybind11/eigen.h>

// needed for vectors
#include <pybind11/stl.h>

#include "PID.hpp"

namespace py = pybind11;
using namespace pid_control;

PYBIND11_MODULE(pid_control_py, m) {
    py::class_<PID>(m, "PID")
        .def(py::init<>())
        .def("restart", &PID::restart)
        .def("cancelRestart", &PID::cancelRestart)
        .def("configParameters", &PID::configParameters)
        .def("update", &PID::update, py::arg("inputs"), py::arg("check_dim") = true)
    ;

    py::class_<PID::ParallelSettings>(m, "PIDSettings")
        // constructor
        .def(py::init<>())
        // methods
        .def("__eq__", &PID::ParallelSettings::operator==)
        .def("__ne__", &PID::ParallelSettings::operator!=)
        // public member variables
        .def_readwrite("Ts",   &PID::ParallelSettings::Ts)
        .def_readwrite("Kp",   &PID::ParallelSettings::Kp)
        .def_readwrite("Ki",   &PID::ParallelSettings::Ki)
        .def_readwrite("Kd",   &PID::ParallelSettings::Kd)
        .def_readwrite("N",    &PID::ParallelSettings::N)
        .def_readwrite("B",    &PID::ParallelSettings::B)
        .def_readwrite("Tt",   &PID::ParallelSettings::Tt)
        .def_readwrite("YMin", &PID::ParallelSettings::YMin)
        .def_readwrite("YMax", &PID::ParallelSettings::YMax)
    ;

    py::class_<PID::Parameters>(m, "PIDParameters")
        // constructor
        .def(py::init<>())
        // methods
        .def("setSettings", &PID::Parameters::setSettings)
        // public member variables
        .def_readwrite("integration_method", &PID::Parameters::integration_method)
        .def_readwrite("wrap", &PID::Parameters::wrap_2pi)
        .def_readwrite("saturate", &PID::Parameters::saturate)
    ;

    py::class_<PID::Inputs>(m, "PIDInput")
        // constructor
        .def(py::init<>())
        // methods
        .def("checkDimensions", &PID::Inputs::checkDimensions)
        .def("resize", &PID::Inputs::resize)
        // static methods
        .def_static("create", &PID::Inputs::create)
        // public member variables
        .def_readwrite("time", &PID::Inputs::time)
        .def_readwrite("ref", &PID::Inputs::reference)
        .def_readwrite("sig", &PID::Inputs::signal)
        .def_readwrite("dref", &PID::Inputs::dotreference)
        .def_readwrite("dsig", &PID::Inputs::dotsignal)
        .def_readwrite("sat", &PID::Inputs::pid_sat)
    ;

    py::class_<PID::Outputs>(m, "PIDOutput")
        // constructor
        .def(py::init<>())
        // methods
        .def("resize", &PID::Outputs::resize)
        .def("getValue", &PID::Outputs::getValue)
    ;
}
