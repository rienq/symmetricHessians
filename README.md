# symmetricHessians

This repository contains supporting material for the paper: "Efficient Symmetric Hessian Propagation for Direct Optimal Control".
The code allows one to reconstruct the presented numerical results for the periodic optimal control on a continuous bioreactor for culture fermentation. The numerical results have been obtained using the open-source ACADO code generation tool on a standard computer, equipped with Intel i7-3720QM processor, and using a $64$-bit version of Ubuntu $14.04$ and the g++ compiler version $4.8.4$.

## Instructions

First compile the *master* branch of ACADO which is a submodule of this repository (following the instructions on http://www.acadotoolkit.org).
To export the C-code, run the following commands:
mkdir build
cd build
cmake ..
make clean all
cd ..
./NMPC

The generated code can be compiled and the simulations can be run by calling "Simulate.m" in MATLAB.
