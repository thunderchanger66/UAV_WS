#include <pybind11/pybind11.h>
#include "ladrc.hpp"

namespace py = pybind11;

PYBIND11_MODULE(ladrc, m) {
    py::class_<LADRC>(m, "LADRC")
        .def(py::init<double, double, double, double>(),
            py::arg("b0"), py::arg("wc"), py::arg("wo"), py::arg("h"))
        .def("step", &LADRC::step, 
            py::arg("ref"), py::arg("y"));
}