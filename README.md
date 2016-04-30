# symmetricHessians

This repository contains supporting material for the paper: "Efficient Symmetric Hessian Propagation for Direct Optimal Control".
The code allows one to reconstruct the presented numerical results for the periodic optimal control on a continuous bioreactor for culture fermentation. The numerical results have been obtained using the open-source ACADO code generation tool on a standard computer, equipped with Intel i7-3720QM processor, and using a 64-bit version of Ubuntu 14.04 and the g++ compiler version 4.8.4.

## Instructions

First install the Matlab interface of the ACADO Toolkit which is a submodule of this repository (follow the instructions on http://acado.github.io/matlab_overview.html to install ACADO from its Matlab interface).


The ACADO code can be generated and compiled and the simulations can be run simply by calling "Simulate.m" from Matlab.
