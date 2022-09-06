#include <dynamic-graph/python/module.hh>

#include "sot-talos-device.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph.sot.core.wrap");

  dg::python::exposeEntity<SoTTalosDevice, bp::bases<dg::sot::Device> >();
}
