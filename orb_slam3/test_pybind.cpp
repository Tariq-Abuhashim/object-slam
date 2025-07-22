#include <iostream>
#include <pybind11/embed.h>
namespace py = pybind11;

int main() {
    py::scoped_interpreter guard{};
    py::gil_scoped_acquire gil;
    py::module torch = py::module::import("torch");
    auto cuda = torch.attr("cuda");
    std::cout << "CUDA available: " << cuda.attr("is_available")().cast<bool>() << std::endl;
}
