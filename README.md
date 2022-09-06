# sot-talos

[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/sot-talos/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/sot-talos/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/sot-talos/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/stack-of-tasks/sot-talos/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/sot-talos/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/sot-talos)

This packages provides a generic Stack Of Tasks library
for the humanoid robot Talos. This library is highly
portable and can be used in various simulators, and
the robot itself.

## Install

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.

## To use:
To embed this library an example is provided in:
sot/core/tools/test_abstract_interface.cpp
