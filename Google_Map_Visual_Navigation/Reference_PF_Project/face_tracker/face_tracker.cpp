#include "face_tracker.hpp"

PYBIND11_MODULE(face_tracker, m) { // module name must match file name
    // optional module docstring
    m.doc() = "A color based  object tracker using particle filter";
    py::class_<FaceTrackerPF>(m, "face_tracker_pf")
        .def(py::init<const py::dict&>(), py::arg("inputs"))
        .def("run_one_iteration", &FaceTrackerPF::run_one_iteration);
}
