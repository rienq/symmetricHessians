/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;
/* Vector of auxiliary variables; number of elements: 2. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((xd[1])*(xd[1]));
a[1] = ((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])/(((real_t)(1.2000000000000000e+00)+xd[1])+(a[0]/(real_t)(2.2000000000000000e+01))));

/* Compute outputs: */
out[0] = (((real_t)(-1.4999999999999999e-01)*xd[0])+(a[1]*xd[0]));
out[1] = (((real_t)(1.4999999999999999e-01)*(u[0]-xd[1]))-((a[1]/(real_t)(4.0000000000000002e-01))*xd[0]));
out[2] = (((real_t)(-1.4999999999999999e-01)*xd[2])+((((real_t)(2.2000000000000002e+00)*a[1])+(real_t)(2.0000000000000001e-01))*xd[0]));
out[3] = xd[0];
out[4] = u[0];
out[5] = (((real_t)(1.4999999999999999e-01)*xd[2])/(real_t)(4.8000000000000000e+01));
}



void acado_acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 15. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((xd[1])*(xd[1]));
a[1] = ((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])/(((real_t)(1.2000000000000000e+00)+xd[1])+(a[0]/(real_t)(2.2000000000000000e+01))));
a[2] = ((real_t)(1.0000000000000000e+00)/(((real_t)(1.2000000000000000e+00)+xd[1])+(a[0]/(real_t)(2.2000000000000000e+01))));
a[3] = ((real_t)(2.0000000000000000e+00)*xd[1]);
a[4] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.2000000000000000e+01));
a[5] = (a[2]*a[2]);
a[6] = ((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[2])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*((real_t)(1.0000000000000000e+00)+(a[3]*a[4])))*a[5]));
a[7] = ((real_t)(1.0000000000000000e+00)/(real_t)(5.0000000000000000e+01));
a[8] = ((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-a[7]))*xd[1])*a[2]);
a[9] = ((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[2])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*((real_t)(1.0000000000000000e+00)+(a[3]*a[4])))*a[5]));
a[10] = ((real_t)(1.0000000000000000e+00)/(real_t)(4.0000000000000002e-01));
a[11] = ((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-a[7]))*xd[1])*a[2]);
a[12] = ((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[2])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*((real_t)(1.0000000000000000e+00)+(a[3]*a[4])))*a[5]));
a[13] = ((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-a[7]))*xd[1])*a[2]);
a[14] = ((real_t)(1.0000000000000000e+00)/(real_t)(4.8000000000000000e+01));

/* Compute outputs: */
out[0] = ((real_t)(-1.4999999999999999e-01)+a[1]);
out[1] = (a[6]*xd[0]);
out[2] = (a[8]*xd[0]);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = ((real_t)(0.0000000000000000e+00)-(a[1]/(real_t)(4.0000000000000002e-01)));
out[8] = (((real_t)(-1.4999999999999999e-01))-((a[9]*a[10])*xd[0]));
out[9] = ((real_t)(0.0000000000000000e+00)-((a[11]*a[10])*xd[0]));
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(1.4999999999999999e-01);
out[14] = (((real_t)(2.2000000000000002e+00)*a[1])+(real_t)(2.0000000000000001e-01));
out[15] = (((real_t)(2.2000000000000002e+00)*a[12])*xd[0]);
out[16] = ((real_t)(-1.4999999999999999e-01)+(((real_t)(2.2000000000000002e+00)*a[13])*xd[0]));
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(1.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(1.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = ((real_t)(1.4999999999999999e-01)*a[14]);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
}





void acado_acado_backward(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 155. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((xd[1])*(xd[1]));
a[1] = ((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])/(((real_t)(1.2000000000000000e+00)+xd[1])+(a[0]/(real_t)(2.2000000000000000e+01))));
a[2] = ((real_t)(0.0000000000000000e+00)-xd[49]);
a[3] = ((real_t)(2.0000000000000000e+00)*xd[1]);
a[4] = ((real_t)(1.0000000000000000e+00)/(((real_t)(1.2000000000000000e+00)+xd[1])+(a[0]/(real_t)(2.2000000000000000e+01))));
a[5] = (a[4]*a[4]);
a[6] = ((real_t)(0.0000000000000000e+00)-((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[5]));
a[7] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.2000000000000000e+01));
a[8] = ((a[3]*(a[6]*a[7]))+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[4])+a[6]));
a[9] = (xd[0]*a[2]);
a[10] = ((real_t)(1.0000000000000000e+00)/(real_t)(4.0000000000000002e-01));
a[11] = ((real_t)(1.4999999999999999e-01)*xd[49]);
a[12] = (xd[0]*xd[50]);
a[13] = (xd[1]*a[4]);
a[14] = ((real_t)(4.7999999999999998e-01)*a[13]);
a[15] = ((real_t)(0.0000000000000000e+00)-a[14]);
a[16] = ((real_t)(1.0000000000000000e+00)/(real_t)(5.0000000000000000e+01));
a[17] = (a[15]*a[16]);
a[18] = ((real_t)(1.0000000000000000e+00)/(real_t)(4.8000000000000000e+01));
a[19] = (xd[53]*a[18]);
a[20] = (a[3]*xd[13]);
a[21] = ((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[20]*a[16])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[13]))*a[4])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*(xd[13]+(a[20]*a[7])))*a[5]));
a[22] = ((real_t)(1.0000000000000000e+00)/(real_t)(4.0000000000000002e-01));
a[23] = (a[3]*xd[14]);
a[24] = ((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[21]*a[16])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[14]))*a[4])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*(xd[14]+(a[23]*a[7])))*a[5]));
a[25] = (a[3]*xd[15]);
a[26] = ((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[22]*a[16])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[15]))*a[4])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*(xd[15]+(a[25]*a[7])))*a[5]));
a[27] = (a[3]*xd[16]);
a[28] = ((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[23]*a[16])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[16]))*a[4])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*(xd[16]+(a[27]*a[7])))*a[5]));
a[29] = (a[3]*xd[17]);
a[30] = ((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[24]*a[16])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[17]))*a[4])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*(xd[17]+(a[29]*a[7])))*a[5]));
a[31] = (a[3]*xd[18]);
a[32] = ((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[25]*a[16])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[18]))*a[4])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*(xd[18]+(a[31]*a[7])))*a[5]));
a[33] = (a[3]*xd[19]);
a[34] = ((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[26]*a[16])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[19]))*a[4])-(((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*(xd[19]+(a[33]*a[7])))*a[5]));
a[35] = ((real_t)(2.0000000000000000e+00)*xd[13]);
a[36] = ((real_t)(1.0000000000000000e+00)/(real_t)(5.0000000000000000e+01));
a[37] = (a[3]*xd[13]);
a[38] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.2000000000000000e+01));
a[39] = ((real_t)(1.0000000000000000e+00)/(((real_t)(1.2000000000000000e+00)+xd[1])+(a[0]/(real_t)(2.2000000000000000e+01))));
a[40] = (a[39]*a[39]);
a[41] = ((real_t)(0.0000000000000000e+00)-((xd[13]+(a[37]*a[38]))*a[40]));
a[42] = ((a[41]*a[4])+(a[4]*a[41]));
a[43] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[20]*a[36])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[13]))*a[5])+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[42])));
a[44] = ((real_t)(1.0000000000000000e+00)/(real_t)(5.0000000000000000e+01));
a[45] = (((a[35]*(a[6]*a[7]))+(a[3]*(a[43]*a[7])))+(((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[20]*a[44])))*a[4])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[41]))+a[43]));
a[46] = (xd[6]*a[2]);
a[47] = (xd[6]*xd[50]);
a[48] = ((real_t)(2.0000000000000000e+00)*xd[14]);
a[49] = (a[3]*xd[14]);
a[50] = ((real_t)(0.0000000000000000e+00)-((xd[14]+(a[49]*a[38]))*a[40]));
a[51] = ((a[50]*a[4])+(a[4]*a[50]));
a[52] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[21]*a[36])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[14]))*a[5])+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[51])));
a[53] = (((a[48]*(a[6]*a[7]))+(a[3]*(a[52]*a[7])))+(((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[21]*a[44])))*a[4])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[50]))+a[52]));
a[54] = (xd[7]*a[2]);
a[55] = (xd[7]*xd[50]);
a[56] = ((real_t)(2.0000000000000000e+00)*xd[15]);
a[57] = (a[3]*xd[15]);
a[58] = ((real_t)(0.0000000000000000e+00)-((xd[15]+(a[57]*a[38]))*a[40]));
a[59] = ((a[58]*a[4])+(a[4]*a[58]));
a[60] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[22]*a[36])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[15]))*a[5])+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[59])));
a[61] = (((a[56]*(a[6]*a[7]))+(a[3]*(a[60]*a[7])))+(((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[22]*a[44])))*a[4])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[58]))+a[60]));
a[62] = (xd[8]*a[2]);
a[63] = (xd[8]*xd[50]);
a[64] = ((real_t)(2.0000000000000000e+00)*xd[16]);
a[65] = (a[3]*xd[16]);
a[66] = ((real_t)(0.0000000000000000e+00)-((xd[16]+(a[65]*a[38]))*a[40]));
a[67] = ((a[66]*a[4])+(a[4]*a[66]));
a[68] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[23]*a[36])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[16]))*a[5])+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[67])));
a[69] = (((a[64]*(a[6]*a[7]))+(a[3]*(a[68]*a[7])))+(((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[23]*a[44])))*a[4])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[66]))+a[68]));
a[70] = (xd[9]*a[2]);
a[71] = (xd[9]*xd[50]);
a[72] = ((real_t)(2.0000000000000000e+00)*xd[17]);
a[73] = (a[3]*xd[17]);
a[74] = ((real_t)(0.0000000000000000e+00)-((xd[17]+(a[73]*a[38]))*a[40]));
a[75] = ((a[74]*a[4])+(a[4]*a[74]));
a[76] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[24]*a[36])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[17]))*a[5])+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[75])));
a[77] = (((a[72]*(a[6]*a[7]))+(a[3]*(a[76]*a[7])))+(((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[24]*a[44])))*a[4])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[74]))+a[76]));
a[78] = (xd[10]*a[2]);
a[79] = (xd[10]*xd[50]);
a[80] = ((real_t)(2.0000000000000000e+00)*xd[18]);
a[81] = (a[3]*xd[18]);
a[82] = ((real_t)(0.0000000000000000e+00)-((xd[18]+(a[81]*a[38]))*a[40]));
a[83] = ((a[82]*a[4])+(a[4]*a[82]));
a[84] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[25]*a[36])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[18]))*a[5])+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[83])));
a[85] = (((a[80]*(a[6]*a[7]))+(a[3]*(a[84]*a[7])))+(((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[25]*a[44])))*a[4])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[82]))+a[84]));
a[86] = (xd[11]*a[2]);
a[87] = (xd[11]*xd[50]);
a[88] = ((real_t)(2.0000000000000000e+00)*xd[19]);
a[89] = (a[3]*xd[19]);
a[90] = ((real_t)(0.0000000000000000e+00)-((xd[19]+(a[89]*a[38]))*a[40]));
a[91] = ((a[90]*a[4])+(a[4]*a[90]));
a[92] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[26]*a[36])))*xd[1])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[19]))*a[5])+((((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*xd[1])*a[91])));
a[93] = (((a[88]*(a[6]*a[7]))+(a[3]*(a[92]*a[7])))+(((((real_t)(4.7999999999999998e-01)*((real_t)(0.0000000000000000e+00)-(xd[26]*a[44])))*a[4])+(((real_t)(4.7999999999999998e-01)*((real_t)(1.0000000000000000e+00)-(xd[2]/(real_t)(5.0000000000000000e+01))))*a[90]))+a[92]));
a[94] = (xd[12]*a[2]);
a[95] = (xd[12]*xd[50]);
a[96] = (a[3]*xd[13]);
a[97] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.2000000000000000e+01));
a[98] = ((real_t)(1.0000000000000000e+00)/(((real_t)(1.2000000000000000e+00)+xd[1])+(a[0]/(real_t)(2.2000000000000000e+01))));
a[99] = (a[98]*a[98]);
a[100] = ((real_t)(0.0000000000000000e+00)-((xd[13]+(a[96]*a[97]))*a[99]));
a[101] = ((xd[13]*a[4])+(xd[1]*a[100]));
a[102] = ((real_t)(4.7999999999999998e-01)*a[101]);
a[103] = ((real_t)(0.0000000000000000e+00)-a[102]);
a[104] = (a[103]*a[16]);
a[105] = (xd[6]*a[2]);
a[106] = (xd[6]*xd[50]);
a[107] = (a[3]*xd[14]);
a[108] = ((real_t)(0.0000000000000000e+00)-((xd[14]+(a[107]*a[97]))*a[99]));
a[109] = ((xd[14]*a[4])+(xd[1]*a[108]));
a[110] = ((real_t)(4.7999999999999998e-01)*a[109]);
a[111] = ((real_t)(0.0000000000000000e+00)-a[110]);
a[112] = (a[111]*a[16]);
a[113] = (xd[7]*a[2]);
a[114] = (xd[7]*xd[50]);
a[115] = (a[3]*xd[15]);
a[116] = ((real_t)(0.0000000000000000e+00)-((xd[15]+(a[115]*a[97]))*a[99]));
a[117] = ((xd[15]*a[4])+(xd[1]*a[116]));
a[118] = ((real_t)(4.7999999999999998e-01)*a[117]);
a[119] = ((real_t)(0.0000000000000000e+00)-a[118]);
a[120] = (a[119]*a[16]);
a[121] = (xd[8]*a[2]);
a[122] = (xd[8]*xd[50]);
a[123] = (a[3]*xd[16]);
a[124] = ((real_t)(0.0000000000000000e+00)-((xd[16]+(a[123]*a[97]))*a[99]));
a[125] = ((xd[16]*a[4])+(xd[1]*a[124]));
a[126] = ((real_t)(4.7999999999999998e-01)*a[125]);
a[127] = ((real_t)(0.0000000000000000e+00)-a[126]);
a[128] = (a[127]*a[16]);
a[129] = (xd[9]*a[2]);
a[130] = (xd[9]*xd[50]);
a[131] = (a[3]*xd[17]);
a[132] = ((real_t)(0.0000000000000000e+00)-((xd[17]+(a[131]*a[97]))*a[99]));
a[133] = ((xd[17]*a[4])+(xd[1]*a[132]));
a[134] = ((real_t)(4.7999999999999998e-01)*a[133]);
a[135] = ((real_t)(0.0000000000000000e+00)-a[134]);
a[136] = (a[135]*a[16]);
a[137] = (xd[10]*a[2]);
a[138] = (xd[10]*xd[50]);
a[139] = (a[3]*xd[18]);
a[140] = ((real_t)(0.0000000000000000e+00)-((xd[18]+(a[139]*a[97]))*a[99]));
a[141] = ((xd[18]*a[4])+(xd[1]*a[140]));
a[142] = ((real_t)(4.7999999999999998e-01)*a[141]);
a[143] = ((real_t)(0.0000000000000000e+00)-a[142]);
a[144] = (a[143]*a[16]);
a[145] = (xd[11]*a[2]);
a[146] = (xd[11]*xd[50]);
a[147] = (a[3]*xd[19]);
a[148] = ((real_t)(0.0000000000000000e+00)-((xd[19]+(a[147]*a[97]))*a[99]));
a[149] = ((xd[19]*a[4])+(xd[1]*a[148]));
a[150] = ((real_t)(4.7999999999999998e-01)*a[149]);
a[151] = ((real_t)(0.0000000000000000e+00)-a[150]);
a[152] = (a[151]*a[16]);
a[153] = (xd[12]*a[2]);
a[154] = (xd[12]*xd[50]);

/* Compute outputs: */
out[0] = ((((((real_t)(-1.4999999999999999e-01)*xd[48])+(a[1]*xd[48]))+((a[1]/(real_t)(4.0000000000000002e-01))*a[2]))+((((real_t)(2.2000000000000002e+00)*a[1])+(real_t)(2.0000000000000001e-01))*xd[50]))+xd[51]);
out[1] = (((a[8]*(xd[0]*xd[48]))+((a[8]*(a[9]*a[10]))+((real_t)(0.0000000000000000e+00)-a[11])))+(a[8]*((real_t)(2.2000000000000002e+00)*a[12])));
out[2] = ((((a[17]*(xd[0]*xd[48]))+(a[17]*(a[9]*a[10])))+((a[17]*((real_t)(2.2000000000000002e+00)*a[12]))+((real_t)(-1.4999999999999999e-01)*xd[50])))+((real_t)(1.4999999999999999e-01)*a[19]));
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (((a[21]*xd[48])+((a[21]*a[22])*a[2]))+(((real_t)(2.2000000000000002e+00)*a[21])*xd[50]));
out[7] = (((a[24]*xd[48])+((a[24]*a[22])*a[2]))+(((real_t)(2.2000000000000002e+00)*a[24])*xd[50]));
out[8] = (((a[26]*xd[48])+((a[26]*a[22])*a[2]))+(((real_t)(2.2000000000000002e+00)*a[26])*xd[50]));
out[9] = (((a[28]*xd[48])+((a[28]*a[22])*a[2]))+(((real_t)(2.2000000000000002e+00)*a[28])*xd[50]));
out[10] = (((a[30]*xd[48])+((a[30]*a[22])*a[2]))+(((real_t)(2.2000000000000002e+00)*a[30])*xd[50]));
out[11] = (((a[32]*xd[48])+((a[32]*a[22])*a[2]))+(((real_t)(2.2000000000000002e+00)*a[32])*xd[50]));
out[12] = (((a[34]*xd[48])+((a[34]*a[22])*a[2]))+(((real_t)(2.2000000000000002e+00)*a[34])*xd[50]));
out[13] = ((((a[45]*(xd[0]*xd[48]))+(a[8]*(xd[6]*xd[48])))+((a[45]*(a[9]*a[10]))+(a[8]*(a[46]*a[10]))))+((a[45]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[8]*((real_t)(2.2000000000000002e+00)*a[47]))));
out[14] = ((((a[53]*(xd[0]*xd[48]))+(a[8]*(xd[7]*xd[48])))+((a[53]*(a[9]*a[10]))+(a[8]*(a[54]*a[10]))))+((a[53]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[8]*((real_t)(2.2000000000000002e+00)*a[55]))));
out[15] = ((((a[61]*(xd[0]*xd[48]))+(a[8]*(xd[8]*xd[48])))+((a[61]*(a[9]*a[10]))+(a[8]*(a[62]*a[10]))))+((a[61]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[8]*((real_t)(2.2000000000000002e+00)*a[63]))));
out[16] = ((((a[69]*(xd[0]*xd[48]))+(a[8]*(xd[9]*xd[48])))+((a[69]*(a[9]*a[10]))+(a[8]*(a[70]*a[10]))))+((a[69]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[8]*((real_t)(2.2000000000000002e+00)*a[71]))));
out[17] = ((((a[77]*(xd[0]*xd[48]))+(a[8]*(xd[10]*xd[48])))+((a[77]*(a[9]*a[10]))+(a[8]*(a[78]*a[10]))))+((a[77]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[8]*((real_t)(2.2000000000000002e+00)*a[79]))));
out[18] = ((((a[85]*(xd[0]*xd[48]))+(a[8]*(xd[11]*xd[48])))+((a[85]*(a[9]*a[10]))+(a[8]*(a[86]*a[10]))))+((a[85]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[8]*((real_t)(2.2000000000000002e+00)*a[87]))));
out[19] = ((((a[93]*(xd[0]*xd[48]))+(a[8]*(xd[12]*xd[48])))+((a[93]*(a[9]*a[10]))+(a[8]*(a[94]*a[10]))))+((a[93]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[8]*((real_t)(2.2000000000000002e+00)*a[95]))));
out[20] = ((((a[104]*(xd[0]*xd[48]))+(a[17]*(xd[6]*xd[48])))+((a[104]*(a[9]*a[10]))+(a[17]*(a[105]*a[10]))))+((a[104]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[17]*((real_t)(2.2000000000000002e+00)*a[106]))));
out[21] = ((((a[112]*(xd[0]*xd[48]))+(a[17]*(xd[7]*xd[48])))+((a[112]*(a[9]*a[10]))+(a[17]*(a[113]*a[10]))))+((a[112]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[17]*((real_t)(2.2000000000000002e+00)*a[114]))));
out[22] = ((((a[120]*(xd[0]*xd[48]))+(a[17]*(xd[8]*xd[48])))+((a[120]*(a[9]*a[10]))+(a[17]*(a[121]*a[10]))))+((a[120]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[17]*((real_t)(2.2000000000000002e+00)*a[122]))));
out[23] = ((((a[128]*(xd[0]*xd[48]))+(a[17]*(xd[9]*xd[48])))+((a[128]*(a[9]*a[10]))+(a[17]*(a[129]*a[10]))))+((a[128]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[17]*((real_t)(2.2000000000000002e+00)*a[130]))));
out[24] = ((((a[136]*(xd[0]*xd[48]))+(a[17]*(xd[10]*xd[48])))+((a[136]*(a[9]*a[10]))+(a[17]*(a[137]*a[10]))))+((a[136]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[17]*((real_t)(2.2000000000000002e+00)*a[138]))));
out[25] = ((((a[144]*(xd[0]*xd[48]))+(a[17]*(xd[11]*xd[48])))+((a[144]*(a[9]*a[10]))+(a[17]*(a[145]*a[10]))))+((a[144]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[17]*((real_t)(2.2000000000000002e+00)*a[146]))));
out[26] = ((((a[152]*(xd[0]*xd[48]))+(a[17]*(xd[12]*xd[48])))+((a[152]*(a[9]*a[10]))+(a[17]*(a[153]*a[10]))))+((a[152]*((real_t)(2.2000000000000002e+00)*a[12]))+(a[17]*((real_t)(2.2000000000000002e+00)*a[154]))));
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim12_triangular( real_t* const A, real_t* const b )
{

b[11] = b[11]/A[143];
b[10] -= + A[131]*b[11];
b[10] = b[10]/A[130];
b[9] -= + A[119]*b[11];
b[9] -= + A[118]*b[10];
b[9] = b[9]/A[117];
b[8] -= + A[107]*b[11];
b[8] -= + A[106]*b[10];
b[8] -= + A[105]*b[9];
b[8] = b[8]/A[104];
b[7] -= + A[95]*b[11];
b[7] -= + A[94]*b[10];
b[7] -= + A[93]*b[9];
b[7] -= + A[92]*b[8];
b[7] = b[7]/A[91];
b[6] -= + A[83]*b[11];
b[6] -= + A[82]*b[10];
b[6] -= + A[81]*b[9];
b[6] -= + A[80]*b[8];
b[6] -= + A[79]*b[7];
b[6] = b[6]/A[78];
b[5] -= + A[71]*b[11];
b[5] -= + A[70]*b[10];
b[5] -= + A[69]*b[9];
b[5] -= + A[68]*b[8];
b[5] -= + A[67]*b[7];
b[5] -= + A[66]*b[6];
b[5] = b[5]/A[65];
b[4] -= + A[59]*b[11];
b[4] -= + A[58]*b[10];
b[4] -= + A[57]*b[9];
b[4] -= + A[56]*b[8];
b[4] -= + A[55]*b[7];
b[4] -= + A[54]*b[6];
b[4] -= + A[53]*b[5];
b[4] = b[4]/A[52];
b[3] -= + A[47]*b[11];
b[3] -= + A[46]*b[10];
b[3] -= + A[45]*b[9];
b[3] -= + A[44]*b[8];
b[3] -= + A[43]*b[7];
b[3] -= + A[42]*b[6];
b[3] -= + A[41]*b[5];
b[3] -= + A[40]*b[4];
b[3] = b[3]/A[39];
b[2] -= + A[35]*b[11];
b[2] -= + A[34]*b[10];
b[2] -= + A[33]*b[9];
b[2] -= + A[32]*b[8];
b[2] -= + A[31]*b[7];
b[2] -= + A[30]*b[6];
b[2] -= + A[29]*b[5];
b[2] -= + A[28]*b[4];
b[2] -= + A[27]*b[3];
b[2] = b[2]/A[26];
b[1] -= + A[23]*b[11];
b[1] -= + A[22]*b[10];
b[1] -= + A[21]*b[9];
b[1] -= + A[20]*b[8];
b[1] -= + A[19]*b[7];
b[1] -= + A[18]*b[6];
b[1] -= + A[17]*b[5];
b[1] -= + A[16]*b[4];
b[1] -= + A[15]*b[3];
b[1] -= + A[14]*b[2];
b[1] = b[1]/A[13];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim12_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 12; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (11); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*12+i]);
	for( j=(i+1); j < 12; j++ ) {
		temp = fabs(A[j*12+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 12; ++k)
{
	acadoWorkspace.rk_dim12_swap = A[i*12+k];
	A[i*12+k] = A[indexMax*12+k];
	A[indexMax*12+k] = acadoWorkspace.rk_dim12_swap;
}
	acadoWorkspace.rk_dim12_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim12_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*12+i];
	for( j=i+1; j < 12; j++ ) {
		A[j*12+i] = -A[j*12+i]/A[i*12+i];
		for( k=i+1; k < 12; k++ ) {
			A[j*12+k] += A[j*12+i] * A[i*12+k];
		}
		b[j] += A[j*12+i] * b[i];
	}
}
det *= A[143];
det = fabs(det);
acado_solve_dim12_triangular( A, b );
return det;
}

void acado_solve_dim12_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim12_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim12_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim12_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim12_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim12_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim12_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim12_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim12_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim12_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim12_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim12_bPerm[10] = b[rk_perm[10]];
acadoWorkspace.rk_dim12_bPerm[11] = b[rk_perm[11]];
acadoWorkspace.rk_dim12_bPerm[1] += A[12]*acadoWorkspace.rk_dim12_bPerm[0];

acadoWorkspace.rk_dim12_bPerm[2] += A[24]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[2] += A[25]*acadoWorkspace.rk_dim12_bPerm[1];

acadoWorkspace.rk_dim12_bPerm[3] += A[36]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[3] += A[37]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[3] += A[38]*acadoWorkspace.rk_dim12_bPerm[2];

acadoWorkspace.rk_dim12_bPerm[4] += A[48]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[4] += A[49]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[4] += A[50]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[4] += A[51]*acadoWorkspace.rk_dim12_bPerm[3];

acadoWorkspace.rk_dim12_bPerm[5] += A[60]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[5] += A[61]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[5] += A[62]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[5] += A[63]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[5] += A[64]*acadoWorkspace.rk_dim12_bPerm[4];

acadoWorkspace.rk_dim12_bPerm[6] += A[72]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[6] += A[73]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[6] += A[74]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[6] += A[75]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[6] += A[76]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[6] += A[77]*acadoWorkspace.rk_dim12_bPerm[5];

acadoWorkspace.rk_dim12_bPerm[7] += A[84]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[7] += A[85]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[7] += A[86]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[7] += A[87]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[7] += A[88]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[7] += A[89]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[7] += A[90]*acadoWorkspace.rk_dim12_bPerm[6];

acadoWorkspace.rk_dim12_bPerm[8] += A[96]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[8] += A[97]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[8] += A[98]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[8] += A[99]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[8] += A[100]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[8] += A[101]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[8] += A[102]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[8] += A[103]*acadoWorkspace.rk_dim12_bPerm[7];

acadoWorkspace.rk_dim12_bPerm[9] += A[108]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[9] += A[109]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[9] += A[110]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[9] += A[111]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[9] += A[112]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[9] += A[113]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[9] += A[114]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[9] += A[115]*acadoWorkspace.rk_dim12_bPerm[7];
acadoWorkspace.rk_dim12_bPerm[9] += A[116]*acadoWorkspace.rk_dim12_bPerm[8];

acadoWorkspace.rk_dim12_bPerm[10] += A[120]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[10] += A[121]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[10] += A[122]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[10] += A[123]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[10] += A[124]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[10] += A[125]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[10] += A[126]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[10] += A[127]*acadoWorkspace.rk_dim12_bPerm[7];
acadoWorkspace.rk_dim12_bPerm[10] += A[128]*acadoWorkspace.rk_dim12_bPerm[8];
acadoWorkspace.rk_dim12_bPerm[10] += A[129]*acadoWorkspace.rk_dim12_bPerm[9];

acadoWorkspace.rk_dim12_bPerm[11] += A[132]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[11] += A[133]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[11] += A[134]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[11] += A[135]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[11] += A[136]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[11] += A[137]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[11] += A[138]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[11] += A[139]*acadoWorkspace.rk_dim12_bPerm[7];
acadoWorkspace.rk_dim12_bPerm[11] += A[140]*acadoWorkspace.rk_dim12_bPerm[8];
acadoWorkspace.rk_dim12_bPerm[11] += A[141]*acadoWorkspace.rk_dim12_bPerm[9];
acadoWorkspace.rk_dim12_bPerm[11] += A[142]*acadoWorkspace.rk_dim12_bPerm[10];


acado_solve_dim12_triangular( A, acadoWorkspace.rk_dim12_bPerm );
b[0] = acadoWorkspace.rk_dim12_bPerm[0];
b[1] = acadoWorkspace.rk_dim12_bPerm[1];
b[2] = acadoWorkspace.rk_dim12_bPerm[2];
b[3] = acadoWorkspace.rk_dim12_bPerm[3];
b[4] = acadoWorkspace.rk_dim12_bPerm[4];
b[5] = acadoWorkspace.rk_dim12_bPerm[5];
b[6] = acadoWorkspace.rk_dim12_bPerm[6];
b[7] = acadoWorkspace.rk_dim12_bPerm[7];
b[8] = acadoWorkspace.rk_dim12_bPerm[8];
b[9] = acadoWorkspace.rk_dim12_bPerm[9];
b[10] = acadoWorkspace.rk_dim12_bPerm[10];
b[11] = acadoWorkspace.rk_dim12_bPerm[11];
}

void acado_solve_dim12_transpose_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{
int i;
int j;

real_t tmp_var;

acadoWorkspace.rk_dim12_bPerm_trans[0] = b[0];
acadoWorkspace.rk_dim12_bPerm_trans[1] = b[1];
acadoWorkspace.rk_dim12_bPerm_trans[2] = b[2];
acadoWorkspace.rk_dim12_bPerm_trans[3] = b[3];
acadoWorkspace.rk_dim12_bPerm_trans[4] = b[4];
acadoWorkspace.rk_dim12_bPerm_trans[5] = b[5];
acadoWorkspace.rk_dim12_bPerm_trans[6] = b[6];
acadoWorkspace.rk_dim12_bPerm_trans[7] = b[7];
acadoWorkspace.rk_dim12_bPerm_trans[8] = b[8];
acadoWorkspace.rk_dim12_bPerm_trans[9] = b[9];
acadoWorkspace.rk_dim12_bPerm_trans[10] = b[10];
acadoWorkspace.rk_dim12_bPerm_trans[11] = b[11];
for (j = 0; j < 12; ++j)
{
for (i = 0; i < j; ++i)
{
acadoWorkspace.rk_dim12_bPerm_trans[j] -= + A[(i * 12) + (j)]*acadoWorkspace.rk_dim12_bPerm_trans[i];
}
tmp_var = 1.0/A[j*13];
acadoWorkspace.rk_dim12_bPerm_trans[j] = + acadoWorkspace.rk_dim12_bPerm_trans[j]*tmp_var;
}
for (i = 11; -1 < i; --i)
{
for (j = 11; i < j; --j)
{
acadoWorkspace.rk_dim12_bPerm_trans[i] += + A[(j * 12) + (i)]*acadoWorkspace.rk_dim12_bPerm_trans[j];
}
}
for (i = 0; i < 12; ++i)
{
j = rk_perm[i];
b[j] = acadoWorkspace.rk_dim12_bPerm_trans[i];
}
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t Ah_mat[ 4 ] = 
{ 1.2000000000000000e-01, 2.5856406460551018e-01, 
-1.8564064605510175e-02, 1.2000000000000000e-01 };


/* Fixed step size:0.48 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int lRun8;
int lRun9;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[6] = rk_eta[97];
acadoWorkspace.rk_seed[54] = rk_eta[97];

/* ------------ Forward loop ------------: */
for (run = 0; run < 5; ++run)
{
if( run > 0 ) {
for (i = 0; i < 6; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 7] = rk_eta[i * 6 + 12];
acadoWorkspace.rk_diffsPrev2[i * 7 + 1] = rk_eta[i * 6 + 13];
acadoWorkspace.rk_diffsPrev2[i * 7 + 2] = rk_eta[i * 6 + 14];
acadoWorkspace.rk_diffsPrev2[i * 7 + 3] = rk_eta[i * 6 + 15];
acadoWorkspace.rk_diffsPrev2[i * 7 + 4] = rk_eta[i * 6 + 16];
acadoWorkspace.rk_diffsPrev2[i * 7 + 5] = rk_eta[i * 6 + 17];
acadoWorkspace.rk_diffsPrev2[i * 7 + 6] = rk_eta[i + 48];
}
}
else{
acadoWorkspace.rk_diffsPrev2[0] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[8] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[9] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[10] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[11] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[12] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[13] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[14] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[15] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[16] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[17] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[18] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[19] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[20] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[21] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[22] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[23] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[24] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[25] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[26] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[27] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[28] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[29] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[30] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[31] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[32] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[33] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[34] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[35] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[36] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[37] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[38] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[39] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[40] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[41] = 0.0000000000000000e+00;
}
acadoWorkspace.rk_S_traj[run * 42] = acadoWorkspace.rk_diffsPrev2[0];
acadoWorkspace.rk_S_traj[run * 42 + 1] = acadoWorkspace.rk_diffsPrev2[1];
acadoWorkspace.rk_S_traj[run * 42 + 2] = acadoWorkspace.rk_diffsPrev2[2];
acadoWorkspace.rk_S_traj[run * 42 + 3] = acadoWorkspace.rk_diffsPrev2[3];
acadoWorkspace.rk_S_traj[run * 42 + 4] = acadoWorkspace.rk_diffsPrev2[4];
acadoWorkspace.rk_S_traj[run * 42 + 5] = acadoWorkspace.rk_diffsPrev2[5];
acadoWorkspace.rk_S_traj[run * 42 + 6] = acadoWorkspace.rk_diffsPrev2[6];
acadoWorkspace.rk_S_traj[run * 42 + 7] = acadoWorkspace.rk_diffsPrev2[7];
acadoWorkspace.rk_S_traj[run * 42 + 8] = acadoWorkspace.rk_diffsPrev2[8];
acadoWorkspace.rk_S_traj[run * 42 + 9] = acadoWorkspace.rk_diffsPrev2[9];
acadoWorkspace.rk_S_traj[run * 42 + 10] = acadoWorkspace.rk_diffsPrev2[10];
acadoWorkspace.rk_S_traj[run * 42 + 11] = acadoWorkspace.rk_diffsPrev2[11];
acadoWorkspace.rk_S_traj[run * 42 + 12] = acadoWorkspace.rk_diffsPrev2[12];
acadoWorkspace.rk_S_traj[run * 42 + 13] = acadoWorkspace.rk_diffsPrev2[13];
acadoWorkspace.rk_S_traj[run * 42 + 14] = acadoWorkspace.rk_diffsPrev2[14];
acadoWorkspace.rk_S_traj[run * 42 + 15] = acadoWorkspace.rk_diffsPrev2[15];
acadoWorkspace.rk_S_traj[run * 42 + 16] = acadoWorkspace.rk_diffsPrev2[16];
acadoWorkspace.rk_S_traj[run * 42 + 17] = acadoWorkspace.rk_diffsPrev2[17];
acadoWorkspace.rk_S_traj[run * 42 + 18] = acadoWorkspace.rk_diffsPrev2[18];
acadoWorkspace.rk_S_traj[run * 42 + 19] = acadoWorkspace.rk_diffsPrev2[19];
acadoWorkspace.rk_S_traj[run * 42 + 20] = acadoWorkspace.rk_diffsPrev2[20];
acadoWorkspace.rk_S_traj[run * 42 + 21] = acadoWorkspace.rk_diffsPrev2[21];
acadoWorkspace.rk_S_traj[run * 42 + 22] = acadoWorkspace.rk_diffsPrev2[22];
acadoWorkspace.rk_S_traj[run * 42 + 23] = acadoWorkspace.rk_diffsPrev2[23];
acadoWorkspace.rk_S_traj[run * 42 + 24] = acadoWorkspace.rk_diffsPrev2[24];
acadoWorkspace.rk_S_traj[run * 42 + 25] = acadoWorkspace.rk_diffsPrev2[25];
acadoWorkspace.rk_S_traj[run * 42 + 26] = acadoWorkspace.rk_diffsPrev2[26];
acadoWorkspace.rk_S_traj[run * 42 + 27] = acadoWorkspace.rk_diffsPrev2[27];
acadoWorkspace.rk_S_traj[run * 42 + 28] = acadoWorkspace.rk_diffsPrev2[28];
acadoWorkspace.rk_S_traj[run * 42 + 29] = acadoWorkspace.rk_diffsPrev2[29];
acadoWorkspace.rk_S_traj[run * 42 + 30] = acadoWorkspace.rk_diffsPrev2[30];
acadoWorkspace.rk_S_traj[run * 42 + 31] = acadoWorkspace.rk_diffsPrev2[31];
acadoWorkspace.rk_S_traj[run * 42 + 32] = acadoWorkspace.rk_diffsPrev2[32];
acadoWorkspace.rk_S_traj[run * 42 + 33] = acadoWorkspace.rk_diffsPrev2[33];
acadoWorkspace.rk_S_traj[run * 42 + 34] = acadoWorkspace.rk_diffsPrev2[34];
acadoWorkspace.rk_S_traj[run * 42 + 35] = acadoWorkspace.rk_diffsPrev2[35];
acadoWorkspace.rk_S_traj[run * 42 + 36] = acadoWorkspace.rk_diffsPrev2[36];
acadoWorkspace.rk_S_traj[run * 42 + 37] = acadoWorkspace.rk_diffsPrev2[37];
acadoWorkspace.rk_S_traj[run * 42 + 38] = acadoWorkspace.rk_diffsPrev2[38];
acadoWorkspace.rk_S_traj[run * 42 + 39] = acadoWorkspace.rk_diffsPrev2[39];
acadoWorkspace.rk_S_traj[run * 42 + 40] = acadoWorkspace.rk_diffsPrev2[40];
acadoWorkspace.rk_S_traj[run * 42 + 41] = acadoWorkspace.rk_diffsPrev2[41];
tmp_index2 = run * 6;
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = (tmp_index2) + (j);
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2 + 1];
}
acado_acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 42 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
acadoWorkspace.rk_A[tmp_index1 * 12] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 1] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 2] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 3] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 4] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 5] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 5)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 12 + 6] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 7] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 8] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 9] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 10] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 11] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 5)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j + 6)] -= 1.0000000000000000e+00;
}
acado_acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 6] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (run1)] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 6 + 1] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (run1)] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 6 + 2] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (run1)] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 6 + 3] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (run1)] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 6 + 4] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 8) + (run1)] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 6 + 5] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 10) + (run1)] - acadoWorkspace.rk_rhsTemp[5];
}
det = acado_solve_dim12_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (j)] += acadoWorkspace.rk_b[j * 6];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (j)] += acadoWorkspace.rk_b[j * 6 + 1];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (j)] += acadoWorkspace.rk_b[j * 6 + 2];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (j)] += acadoWorkspace.rk_b[j * 6 + 3];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 8) + (j)] += acadoWorkspace.rk_b[j * 6 + 4];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 10) + (j)] += acadoWorkspace.rk_b[j * 6 + 5];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = (tmp_index2) + (j);
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2 + 1];
}
acado_acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 6] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (run1)] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 6 + 1] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (run1)] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 6 + 2] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (run1)] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 6 + 3] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (run1)] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 6 + 4] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 8) + (run1)] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 6 + 5] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 10) + (run1)] - acadoWorkspace.rk_rhsTemp[5];
}
acado_solve_dim12_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (j)] += acadoWorkspace.rk_b[j * 6];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (j)] += acadoWorkspace.rk_b[j * 6 + 1];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (j)] += acadoWorkspace.rk_b[j * 6 + 2];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (j)] += acadoWorkspace.rk_b[j * 6 + 3];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 8) + (j)] += acadoWorkspace.rk_b[j * 6 + 4];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 10) + (j)] += acadoWorkspace.rk_b[j * 6 + 5];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = (tmp_index2) + (j);
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2 + 1];
}
acado_acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 42 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
acadoWorkspace.rk_A[tmp_index1 * 12] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 1] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 2] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 3] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 4] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 5] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 5)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 12 + 6] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 7] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 8] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 9] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 10] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 11] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 42) + (j * 7 + 5)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j + 6)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
tmp_index1 = (tmp_index2 * 2) + (run1 * 6);
i = (tmp_index2) + (j);
acadoWorkspace.rk_stageV_traj[(0) + ((0) + ((tmp_index1) + (j)))] = rk_eta[j];
acadoWorkspace.rk_stageV_traj[(0) + ((0) + ((tmp_index1) + (j)))] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[i * 2];
acadoWorkspace.rk_stageV_traj[(0) + ((0) + ((tmp_index1) + (j)))] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[i * 2 + 1];
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 6] = - acadoWorkspace.rk_diffsTemp2[(i * 42) + (run1)];
acadoWorkspace.rk_b[i * 6 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 42) + (run1 + 7)];
acadoWorkspace.rk_b[i * 6 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 42) + (run1 + 14)];
acadoWorkspace.rk_b[i * 6 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 42) + (run1 + 21)];
acadoWorkspace.rk_b[i * 6 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 42) + (run1 + 28)];
acadoWorkspace.rk_b[i * 6 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 42) + (run1 + 35)];
}
if( 0 == run1 ) {
det = acado_solve_dim12_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
}
 else {
acado_solve_dim12_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
}
for (i = 0; i < 2; ++i)
{
tmp_index1 = (run * 42) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6];
tmp_index1 = (run * 42 + 7) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 1];
tmp_index1 = (run * 42 + 14) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 2];
tmp_index1 = (run * 42 + 21) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 3];
tmp_index1 = (run * 42 + 28) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 4];
tmp_index1 = (run * 42 + 35) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 7) + (run1)] = (i == run1-0);
tmp_index1 = ((run * 6) + (i)) * (7);
tmp_index2 = (tmp_index1) + (run1);
acadoWorkspace.rk_diffsNew2[(i * 7) + (run1)] += + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1]*(real_t)2.3999999999999999e-01;
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index1 = (i * 6) + (j);
tmp_index2 = (run1) + (j * 7);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 42) + (tmp_index2 + 6)];
}
}
acado_solve_dim12_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
for (i = 0; i < 2; ++i)
{
tmp_index1 = (run * 42 + 6) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6];
tmp_index1 = (run * 42 + 13) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 1];
tmp_index1 = (run * 42 + 20) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 2];
tmp_index1 = (run * 42 + 27) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 3];
tmp_index1 = (run * 42 + 34) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 4];
tmp_index1 = (run * 42 + 41) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
tmp_index1 = ((run * 6) + (i)) * (7);
tmp_index2 = (tmp_index1 + 6) + (run1);
acadoWorkspace.rk_diffsNew2[(i * 7) + (run1 + 6)] = + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1]*(real_t)2.3999999999999999e-01;
}
}
for (lRun8 = 0;lRun8 < 12; ++lRun8)
for (lRun9 = 0;lRun9 < 12; ++lRun9)
acadoWorkspace.rk_A_traj[(((lRun8) + (run * 12)) * (12)) + (lRun9)] = acadoWorkspace.rk_A[(lRun8 * 12) + (lRun9)];
acadoWorkspace.rk_aux_traj[run * 12] = acadoWorkspace.rk_dim12_perm[0];
acadoWorkspace.rk_aux_traj[run * 12 + 1] = acadoWorkspace.rk_dim12_perm[1];
acadoWorkspace.rk_aux_traj[run * 12 + 2] = acadoWorkspace.rk_dim12_perm[2];
acadoWorkspace.rk_aux_traj[run * 12 + 3] = acadoWorkspace.rk_dim12_perm[3];
acadoWorkspace.rk_aux_traj[run * 12 + 4] = acadoWorkspace.rk_dim12_perm[4];
acadoWorkspace.rk_aux_traj[run * 12 + 5] = acadoWorkspace.rk_dim12_perm[5];
acadoWorkspace.rk_aux_traj[run * 12 + 6] = acadoWorkspace.rk_dim12_perm[6];
acadoWorkspace.rk_aux_traj[run * 12 + 7] = acadoWorkspace.rk_dim12_perm[7];
acadoWorkspace.rk_aux_traj[run * 12 + 8] = acadoWorkspace.rk_dim12_perm[8];
acadoWorkspace.rk_aux_traj[run * 12 + 9] = acadoWorkspace.rk_dim12_perm[9];
acadoWorkspace.rk_aux_traj[run * 12 + 10] = acadoWorkspace.rk_dim12_perm[10];
acadoWorkspace.rk_aux_traj[run * 12 + 11] = acadoWorkspace.rk_dim12_perm[11];
rk_eta[0] += + acadoWorkspace.rk_Ktraj[run * 12]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_Ktraj[run * 12 + 1]*(real_t)2.3999999999999999e-01;
rk_eta[1] += + acadoWorkspace.rk_Ktraj[run * 12 + 2]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_Ktraj[run * 12 + 3]*(real_t)2.3999999999999999e-01;
rk_eta[2] += + acadoWorkspace.rk_Ktraj[run * 12 + 4]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_Ktraj[run * 12 + 5]*(real_t)2.3999999999999999e-01;
rk_eta[3] += + acadoWorkspace.rk_Ktraj[run * 12 + 6]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_Ktraj[run * 12 + 7]*(real_t)2.3999999999999999e-01;
rk_eta[4] += + acadoWorkspace.rk_Ktraj[run * 12 + 8]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_Ktraj[run * 12 + 9]*(real_t)2.3999999999999999e-01;
rk_eta[5] += + acadoWorkspace.rk_Ktraj[run * 12 + 10]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_Ktraj[run * 12 + 11]*(real_t)2.3999999999999999e-01;
if( run == 0 ) {
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 12] = acadoWorkspace.rk_diffsNew2[(i * 7) + (j)];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 48] = acadoWorkspace.rk_diffsNew2[(i * 7) + (j + 6)];
}
}
}
else {
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 12] = + acadoWorkspace.rk_diffsNew2[i * 7]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 7 + 1]*acadoWorkspace.rk_diffsPrev2[j + 7];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 7 + 2]*acadoWorkspace.rk_diffsPrev2[j + 14];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 7 + 3]*acadoWorkspace.rk_diffsPrev2[j + 21];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 7 + 4]*acadoWorkspace.rk_diffsPrev2[j + 28];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 7 + 5]*acadoWorkspace.rk_diffsPrev2[j + 35];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 48] = acadoWorkspace.rk_diffsNew2[(i * 7) + (j + 6)];
rk_eta[tmp_index2 + 48] += + acadoWorkspace.rk_diffsNew2[i * 7]*acadoWorkspace.rk_diffsPrev2[j + 6];
rk_eta[tmp_index2 + 48] += + acadoWorkspace.rk_diffsNew2[i * 7 + 1]*acadoWorkspace.rk_diffsPrev2[j + 13];
rk_eta[tmp_index2 + 48] += + acadoWorkspace.rk_diffsNew2[i * 7 + 2]*acadoWorkspace.rk_diffsPrev2[j + 20];
rk_eta[tmp_index2 + 48] += + acadoWorkspace.rk_diffsNew2[i * 7 + 3]*acadoWorkspace.rk_diffsPrev2[j + 27];
rk_eta[tmp_index2 + 48] += + acadoWorkspace.rk_diffsNew2[i * 7 + 4]*acadoWorkspace.rk_diffsPrev2[j + 34];
rk_eta[tmp_index2 + 48] += + acadoWorkspace.rk_diffsNew2[i * 7 + 5]*acadoWorkspace.rk_diffsPrev2[j + 41];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 2.0000000000000001e-01;
}
/* ------------ BACKWARD loop ------------: */
rk_eta[54] = 0.0000000000000000e+00;
rk_eta[55] = 0.0000000000000000e+00;
rk_eta[56] = 0.0000000000000000e+00;
rk_eta[57] = 0.0000000000000000e+00;
rk_eta[58] = 0.0000000000000000e+00;
rk_eta[59] = 0.0000000000000000e+00;
rk_eta[60] = 0.0000000000000000e+00;
rk_eta[61] = 0.0000000000000000e+00;
rk_eta[62] = 0.0000000000000000e+00;
rk_eta[63] = 0.0000000000000000e+00;
rk_eta[64] = 0.0000000000000000e+00;
rk_eta[65] = 0.0000000000000000e+00;
rk_eta[66] = 0.0000000000000000e+00;
rk_eta[67] = 0.0000000000000000e+00;
rk_eta[68] = 0.0000000000000000e+00;
rk_eta[69] = 0.0000000000000000e+00;
rk_eta[70] = 0.0000000000000000e+00;
rk_eta[71] = 0.0000000000000000e+00;
rk_eta[72] = 0.0000000000000000e+00;
rk_eta[73] = 0.0000000000000000e+00;
rk_eta[74] = 0.0000000000000000e+00;
rk_eta[75] = 0.0000000000000000e+00;
rk_eta[76] = 0.0000000000000000e+00;
rk_eta[77] = 0.0000000000000000e+00;
rk_eta[78] = 0.0000000000000000e+00;
rk_eta[79] = 0.0000000000000000e+00;
rk_eta[80] = 0.0000000000000000e+00;
rk_eta[81] = 0.0000000000000000e+00;
rk_eta[82] = 0.0000000000000000e+00;
rk_eta[83] = 0.0000000000000000e+00;
rk_eta[84] = 0.0000000000000000e+00;
rk_eta[85] = 0.0000000000000000e+00;
rk_eta[86] = 0.0000000000000000e+00;
rk_eta[87] = 0.0000000000000000e+00;
rk_eta[88] = 0.0000000000000000e+00;
rk_eta[89] = 0.0000000000000000e+00;
rk_eta[90] = 0.0000000000000000e+00;
rk_eta[91] = 0.0000000000000000e+00;
rk_eta[92] = 0.0000000000000000e+00;
rk_eta[93] = 0.0000000000000000e+00;
rk_eta[94] = 0.0000000000000000e+00;
rk_eta[95] = 0.0000000000000000e+00;
rk_eta[96] = 0.0000000000000000e+00;
for (run = 4; -1 < run; --run)
{
acadoWorkspace.rk_b_trans[0] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[0] -= + (real_t)2.3999999999999999e-01*rk_eta[6];
acadoWorkspace.rk_b_trans[1] -= + (real_t)2.3999999999999999e-01*rk_eta[7];
acadoWorkspace.rk_b_trans[2] -= + (real_t)2.3999999999999999e-01*rk_eta[8];
acadoWorkspace.rk_b_trans[3] -= + (real_t)2.3999999999999999e-01*rk_eta[9];
acadoWorkspace.rk_b_trans[4] -= + (real_t)2.3999999999999999e-01*rk_eta[10];
acadoWorkspace.rk_b_trans[5] -= + (real_t)2.3999999999999999e-01*rk_eta[11];
acadoWorkspace.rk_b_trans[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[8] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[9] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[10] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[11] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[6] -= + (real_t)2.3999999999999999e-01*rk_eta[6];
acadoWorkspace.rk_b_trans[7] -= + (real_t)2.3999999999999999e-01*rk_eta[7];
acadoWorkspace.rk_b_trans[8] -= + (real_t)2.3999999999999999e-01*rk_eta[8];
acadoWorkspace.rk_b_trans[9] -= + (real_t)2.3999999999999999e-01*rk_eta[9];
acadoWorkspace.rk_b_trans[10] -= + (real_t)2.3999999999999999e-01*rk_eta[10];
acadoWorkspace.rk_b_trans[11] -= + (real_t)2.3999999999999999e-01*rk_eta[11];
acado_solve_dim12_transpose_reuse( &(acadoWorkspace.rk_A_traj[ run * 144 ]), acadoWorkspace.rk_b_trans, &(acadoWorkspace.rk_aux_traj[ run * 12 ]) );
{ int lCopy; for (lCopy = 0; lCopy < 49; lCopy++) acadoWorkspace.rk_hess1[ lCopy ] = 0; }
acadoWorkspace.rk_seed[0] = acadoWorkspace.rk_stageV_traj[run * 12];
acadoWorkspace.rk_seed[1] = acadoWorkspace.rk_stageV_traj[run * 12 + 1];
acadoWorkspace.rk_seed[2] = acadoWorkspace.rk_stageV_traj[run * 12 + 2];
acadoWorkspace.rk_seed[3] = acadoWorkspace.rk_stageV_traj[run * 12 + 3];
acadoWorkspace.rk_seed[4] = acadoWorkspace.rk_stageV_traj[run * 12 + 4];
acadoWorkspace.rk_seed[5] = acadoWorkspace.rk_stageV_traj[run * 12 + 5];
acadoWorkspace.rk_diffsPrev2[0] = acadoWorkspace.rk_S_traj[run * 42];
acadoWorkspace.rk_diffsPrev2[1] = acadoWorkspace.rk_S_traj[run * 42 + 1];
acadoWorkspace.rk_diffsPrev2[2] = acadoWorkspace.rk_S_traj[run * 42 + 2];
acadoWorkspace.rk_diffsPrev2[3] = acadoWorkspace.rk_S_traj[run * 42 + 3];
acadoWorkspace.rk_diffsPrev2[4] = acadoWorkspace.rk_S_traj[run * 42 + 4];
acadoWorkspace.rk_diffsPrev2[5] = acadoWorkspace.rk_S_traj[run * 42 + 5];
acadoWorkspace.rk_diffsPrev2[6] = acadoWorkspace.rk_S_traj[run * 42 + 6];
acadoWorkspace.rk_diffsPrev2[7] = acadoWorkspace.rk_S_traj[run * 42 + 7];
acadoWorkspace.rk_diffsPrev2[8] = acadoWorkspace.rk_S_traj[run * 42 + 8];
acadoWorkspace.rk_diffsPrev2[9] = acadoWorkspace.rk_S_traj[run * 42 + 9];
acadoWorkspace.rk_diffsPrev2[10] = acadoWorkspace.rk_S_traj[run * 42 + 10];
acadoWorkspace.rk_diffsPrev2[11] = acadoWorkspace.rk_S_traj[run * 42 + 11];
acadoWorkspace.rk_diffsPrev2[12] = acadoWorkspace.rk_S_traj[run * 42 + 12];
acadoWorkspace.rk_diffsPrev2[13] = acadoWorkspace.rk_S_traj[run * 42 + 13];
acadoWorkspace.rk_diffsPrev2[14] = acadoWorkspace.rk_S_traj[run * 42 + 14];
acadoWorkspace.rk_diffsPrev2[15] = acadoWorkspace.rk_S_traj[run * 42 + 15];
acadoWorkspace.rk_diffsPrev2[16] = acadoWorkspace.rk_S_traj[run * 42 + 16];
acadoWorkspace.rk_diffsPrev2[17] = acadoWorkspace.rk_S_traj[run * 42 + 17];
acadoWorkspace.rk_diffsPrev2[18] = acadoWorkspace.rk_S_traj[run * 42 + 18];
acadoWorkspace.rk_diffsPrev2[19] = acadoWorkspace.rk_S_traj[run * 42 + 19];
acadoWorkspace.rk_diffsPrev2[20] = acadoWorkspace.rk_S_traj[run * 42 + 20];
acadoWorkspace.rk_diffsPrev2[21] = acadoWorkspace.rk_S_traj[run * 42 + 21];
acadoWorkspace.rk_diffsPrev2[22] = acadoWorkspace.rk_S_traj[run * 42 + 22];
acadoWorkspace.rk_diffsPrev2[23] = acadoWorkspace.rk_S_traj[run * 42 + 23];
acadoWorkspace.rk_diffsPrev2[24] = acadoWorkspace.rk_S_traj[run * 42 + 24];
acadoWorkspace.rk_diffsPrev2[25] = acadoWorkspace.rk_S_traj[run * 42 + 25];
acadoWorkspace.rk_diffsPrev2[26] = acadoWorkspace.rk_S_traj[run * 42 + 26];
acadoWorkspace.rk_diffsPrev2[27] = acadoWorkspace.rk_S_traj[run * 42 + 27];
acadoWorkspace.rk_diffsPrev2[28] = acadoWorkspace.rk_S_traj[run * 42 + 28];
acadoWorkspace.rk_diffsPrev2[29] = acadoWorkspace.rk_S_traj[run * 42 + 29];
acadoWorkspace.rk_diffsPrev2[30] = acadoWorkspace.rk_S_traj[run * 42 + 30];
acadoWorkspace.rk_diffsPrev2[31] = acadoWorkspace.rk_S_traj[run * 42 + 31];
acadoWorkspace.rk_diffsPrev2[32] = acadoWorkspace.rk_S_traj[run * 42 + 32];
acadoWorkspace.rk_diffsPrev2[33] = acadoWorkspace.rk_S_traj[run * 42 + 33];
acadoWorkspace.rk_diffsPrev2[34] = acadoWorkspace.rk_S_traj[run * 42 + 34];
acadoWorkspace.rk_diffsPrev2[35] = acadoWorkspace.rk_S_traj[run * 42 + 35];
acadoWorkspace.rk_diffsPrev2[36] = acadoWorkspace.rk_S_traj[run * 42 + 36];
acadoWorkspace.rk_diffsPrev2[37] = acadoWorkspace.rk_S_traj[run * 42 + 37];
acadoWorkspace.rk_diffsPrev2[38] = acadoWorkspace.rk_S_traj[run * 42 + 38];
acadoWorkspace.rk_diffsPrev2[39] = acadoWorkspace.rk_S_traj[run * 42 + 39];
acadoWorkspace.rk_diffsPrev2[40] = acadoWorkspace.rk_S_traj[run * 42 + 40];
acadoWorkspace.rk_diffsPrev2[41] = acadoWorkspace.rk_S_traj[run * 42 + 41];
for (i = 0; i < 6; ++i)
{
tmp_index1 = ((run * 6) + (i)) * (7);
for (j = 0; j < 7; ++j)
{
tmp_index2 = (tmp_index1) + (j);
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[0]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2];
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[1]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1];
}
}
acadoWorkspace.rk_seed[6] = acadoWorkspace.rk_diffsPrev2[0];
acadoWorkspace.rk_seed[7] = acadoWorkspace.rk_diffsPrev2[1];
acadoWorkspace.rk_seed[8] = acadoWorkspace.rk_diffsPrev2[2];
acadoWorkspace.rk_seed[9] = acadoWorkspace.rk_diffsPrev2[3];
acadoWorkspace.rk_seed[10] = acadoWorkspace.rk_diffsPrev2[4];
acadoWorkspace.rk_seed[11] = acadoWorkspace.rk_diffsPrev2[5];
acadoWorkspace.rk_seed[12] = acadoWorkspace.rk_diffsPrev2[6];
acadoWorkspace.rk_seed[13] = acadoWorkspace.rk_diffsPrev2[7];
acadoWorkspace.rk_seed[14] = acadoWorkspace.rk_diffsPrev2[8];
acadoWorkspace.rk_seed[15] = acadoWorkspace.rk_diffsPrev2[9];
acadoWorkspace.rk_seed[16] = acadoWorkspace.rk_diffsPrev2[10];
acadoWorkspace.rk_seed[17] = acadoWorkspace.rk_diffsPrev2[11];
acadoWorkspace.rk_seed[18] = acadoWorkspace.rk_diffsPrev2[12];
acadoWorkspace.rk_seed[19] = acadoWorkspace.rk_diffsPrev2[13];
acadoWorkspace.rk_seed[20] = acadoWorkspace.rk_diffsPrev2[14];
acadoWorkspace.rk_seed[21] = acadoWorkspace.rk_diffsPrev2[15];
acadoWorkspace.rk_seed[22] = acadoWorkspace.rk_diffsPrev2[16];
acadoWorkspace.rk_seed[23] = acadoWorkspace.rk_diffsPrev2[17];
acadoWorkspace.rk_seed[24] = acadoWorkspace.rk_diffsPrev2[18];
acadoWorkspace.rk_seed[25] = acadoWorkspace.rk_diffsPrev2[19];
acadoWorkspace.rk_seed[26] = acadoWorkspace.rk_diffsPrev2[20];
acadoWorkspace.rk_seed[27] = acadoWorkspace.rk_diffsPrev2[21];
acadoWorkspace.rk_seed[28] = acadoWorkspace.rk_diffsPrev2[22];
acadoWorkspace.rk_seed[29] = acadoWorkspace.rk_diffsPrev2[23];
acadoWorkspace.rk_seed[30] = acadoWorkspace.rk_diffsPrev2[24];
acadoWorkspace.rk_seed[31] = acadoWorkspace.rk_diffsPrev2[25];
acadoWorkspace.rk_seed[32] = acadoWorkspace.rk_diffsPrev2[26];
acadoWorkspace.rk_seed[33] = acadoWorkspace.rk_diffsPrev2[27];
acadoWorkspace.rk_seed[34] = acadoWorkspace.rk_diffsPrev2[28];
acadoWorkspace.rk_seed[35] = acadoWorkspace.rk_diffsPrev2[29];
acadoWorkspace.rk_seed[36] = acadoWorkspace.rk_diffsPrev2[30];
acadoWorkspace.rk_seed[37] = acadoWorkspace.rk_diffsPrev2[31];
acadoWorkspace.rk_seed[38] = acadoWorkspace.rk_diffsPrev2[32];
acadoWorkspace.rk_seed[39] = acadoWorkspace.rk_diffsPrev2[33];
acadoWorkspace.rk_seed[40] = acadoWorkspace.rk_diffsPrev2[34];
acadoWorkspace.rk_seed[41] = acadoWorkspace.rk_diffsPrev2[35];
acadoWorkspace.rk_seed[42] = acadoWorkspace.rk_diffsPrev2[36];
acadoWorkspace.rk_seed[43] = acadoWorkspace.rk_diffsPrev2[37];
acadoWorkspace.rk_seed[44] = acadoWorkspace.rk_diffsPrev2[38];
acadoWorkspace.rk_seed[45] = acadoWorkspace.rk_diffsPrev2[39];
acadoWorkspace.rk_seed[46] = acadoWorkspace.rk_diffsPrev2[40];
acadoWorkspace.rk_seed[47] = acadoWorkspace.rk_diffsPrev2[41];
acadoWorkspace.rk_seed[48] = acadoWorkspace.rk_b_trans[0];
acadoWorkspace.rk_seed[49] = acadoWorkspace.rk_b_trans[1];
acadoWorkspace.rk_seed[50] = acadoWorkspace.rk_b_trans[2];
acadoWorkspace.rk_seed[51] = acadoWorkspace.rk_b_trans[3];
acadoWorkspace.rk_seed[52] = acadoWorkspace.rk_b_trans[4];
acadoWorkspace.rk_seed[53] = acadoWorkspace.rk_b_trans[5];
acado_acado_backward( acadoWorkspace.rk_seed, acadoWorkspace.rk_adjoint );
rk_eta[6] += acadoWorkspace.rk_adjoint[0];
rk_eta[7] += acadoWorkspace.rk_adjoint[1];
rk_eta[8] += acadoWorkspace.rk_adjoint[2];
rk_eta[9] += acadoWorkspace.rk_adjoint[3];
rk_eta[10] += acadoWorkspace.rk_adjoint[4];
rk_eta[11] += acadoWorkspace.rk_adjoint[5];
acadoWorkspace.rk_hess2[0] = acadoWorkspace.rk_adjoint[6];
acadoWorkspace.rk_hess2[1] = acadoWorkspace.rk_adjoint[7];
acadoWorkspace.rk_hess2[2] = acadoWorkspace.rk_adjoint[8];
acadoWorkspace.rk_hess2[3] = acadoWorkspace.rk_adjoint[9];
acadoWorkspace.rk_hess2[4] = acadoWorkspace.rk_adjoint[10];
acadoWorkspace.rk_hess2[5] = acadoWorkspace.rk_adjoint[11];
acadoWorkspace.rk_hess2[6] = acadoWorkspace.rk_adjoint[12];
acadoWorkspace.rk_hess2[7] = acadoWorkspace.rk_adjoint[13];
acadoWorkspace.rk_hess2[8] = acadoWorkspace.rk_adjoint[14];
acadoWorkspace.rk_hess2[9] = acadoWorkspace.rk_adjoint[15];
acadoWorkspace.rk_hess2[10] = acadoWorkspace.rk_adjoint[16];
acadoWorkspace.rk_hess2[11] = acadoWorkspace.rk_adjoint[17];
acadoWorkspace.rk_hess2[12] = acadoWorkspace.rk_adjoint[18];
acadoWorkspace.rk_hess2[13] = acadoWorkspace.rk_adjoint[19];
acadoWorkspace.rk_hess2[14] = acadoWorkspace.rk_adjoint[20];
acadoWorkspace.rk_hess2[15] = acadoWorkspace.rk_adjoint[21];
acadoWorkspace.rk_hess2[16] = acadoWorkspace.rk_adjoint[22];
acadoWorkspace.rk_hess2[17] = acadoWorkspace.rk_adjoint[23];
acadoWorkspace.rk_hess2[18] = acadoWorkspace.rk_adjoint[24];
acadoWorkspace.rk_hess2[19] = acadoWorkspace.rk_adjoint[25];
acadoWorkspace.rk_hess2[20] = acadoWorkspace.rk_adjoint[26];
acadoWorkspace.rk_hess2[21] = acadoWorkspace.rk_adjoint[27];
acadoWorkspace.rk_hess2[22] = acadoWorkspace.rk_adjoint[28];
acadoWorkspace.rk_hess2[23] = acadoWorkspace.rk_adjoint[29];
acadoWorkspace.rk_hess2[24] = acadoWorkspace.rk_adjoint[30];
acadoWorkspace.rk_hess2[25] = acadoWorkspace.rk_adjoint[31];
acadoWorkspace.rk_hess2[26] = acadoWorkspace.rk_adjoint[32];
acadoWorkspace.rk_hess2[27] = acadoWorkspace.rk_adjoint[33];
acadoWorkspace.rk_hess2[28] = acadoWorkspace.rk_adjoint[34];
acadoWorkspace.rk_hess2[29] = acadoWorkspace.rk_adjoint[35];
acadoWorkspace.rk_hess2[30] = acadoWorkspace.rk_adjoint[36];
acadoWorkspace.rk_hess2[31] = acadoWorkspace.rk_adjoint[37];
acadoWorkspace.rk_hess2[32] = acadoWorkspace.rk_adjoint[38];
acadoWorkspace.rk_hess2[33] = acadoWorkspace.rk_adjoint[39];
acadoWorkspace.rk_hess2[34] = acadoWorkspace.rk_adjoint[40];
acadoWorkspace.rk_hess2[35] = acadoWorkspace.rk_adjoint[41];
acadoWorkspace.rk_hess2[36] = acadoWorkspace.rk_adjoint[42];
acadoWorkspace.rk_hess2[37] = acadoWorkspace.rk_adjoint[43];
acadoWorkspace.rk_hess2[38] = acadoWorkspace.rk_adjoint[44];
acadoWorkspace.rk_hess2[39] = acadoWorkspace.rk_adjoint[45];
acadoWorkspace.rk_hess2[40] = acadoWorkspace.rk_adjoint[46];
acadoWorkspace.rk_hess2[41] = acadoWorkspace.rk_adjoint[47];
acadoWorkspace.rk_hess2[42] = acadoWorkspace.rk_adjoint[48];
acadoWorkspace.rk_hess2[43] = acadoWorkspace.rk_adjoint[49];
acadoWorkspace.rk_hess2[44] = acadoWorkspace.rk_adjoint[50];
acadoWorkspace.rk_hess2[45] = acadoWorkspace.rk_adjoint[51];
acadoWorkspace.rk_hess2[46] = acadoWorkspace.rk_adjoint[52];
acadoWorkspace.rk_hess2[47] = acadoWorkspace.rk_adjoint[53];
acadoWorkspace.rk_hess2[48] = acadoWorkspace.rk_adjoint[54];
acadoWorkspace.rk_diffsPrev2[0] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[8] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[9] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[10] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[11] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[12] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[13] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[14] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[15] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[16] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[17] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[18] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[19] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[20] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[21] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[22] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[23] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[24] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[25] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[26] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[27] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[28] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[29] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[30] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[31] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[32] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[33] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[34] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[35] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[36] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[37] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[38] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[39] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[40] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[41] = 0.0000000000000000e+00;
for (i = 0; i < 6; ++i)
{
tmp_index1 = ((run * 6) + (i)) * (7);
for (j = 0; j < 7; ++j)
{
tmp_index2 = (tmp_index1) + (j);
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[0]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2];
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[1]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1];
}
}
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i]*acadoWorkspace.rk_hess2[j];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 7]*acadoWorkspace.rk_hess2[j + 7];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 14]*acadoWorkspace.rk_hess2[j + 14];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 21]*acadoWorkspace.rk_hess2[j + 21];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 28]*acadoWorkspace.rk_hess2[j + 28];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 35]*acadoWorkspace.rk_hess2[j + 35];
}
}
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += acadoWorkspace.rk_hess2[(i * 7 + 42) + (j)];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 6]*acadoWorkspace.rk_hess2[j];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 13]*acadoWorkspace.rk_hess2[j + 7];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 20]*acadoWorkspace.rk_hess2[j + 14];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 27]*acadoWorkspace.rk_hess2[j + 21];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 34]*acadoWorkspace.rk_hess2[j + 28];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 41]*acadoWorkspace.rk_hess2[j + 35];
}
}
acadoWorkspace.rk_seed[0] = acadoWorkspace.rk_stageV_traj[run * 12 + 6];
acadoWorkspace.rk_seed[1] = acadoWorkspace.rk_stageV_traj[run * 12 + 7];
acadoWorkspace.rk_seed[2] = acadoWorkspace.rk_stageV_traj[run * 12 + 8];
acadoWorkspace.rk_seed[3] = acadoWorkspace.rk_stageV_traj[run * 12 + 9];
acadoWorkspace.rk_seed[4] = acadoWorkspace.rk_stageV_traj[run * 12 + 10];
acadoWorkspace.rk_seed[5] = acadoWorkspace.rk_stageV_traj[run * 12 + 11];
acadoWorkspace.rk_diffsPrev2[0] = acadoWorkspace.rk_S_traj[run * 42];
acadoWorkspace.rk_diffsPrev2[1] = acadoWorkspace.rk_S_traj[run * 42 + 1];
acadoWorkspace.rk_diffsPrev2[2] = acadoWorkspace.rk_S_traj[run * 42 + 2];
acadoWorkspace.rk_diffsPrev2[3] = acadoWorkspace.rk_S_traj[run * 42 + 3];
acadoWorkspace.rk_diffsPrev2[4] = acadoWorkspace.rk_S_traj[run * 42 + 4];
acadoWorkspace.rk_diffsPrev2[5] = acadoWorkspace.rk_S_traj[run * 42 + 5];
acadoWorkspace.rk_diffsPrev2[6] = acadoWorkspace.rk_S_traj[run * 42 + 6];
acadoWorkspace.rk_diffsPrev2[7] = acadoWorkspace.rk_S_traj[run * 42 + 7];
acadoWorkspace.rk_diffsPrev2[8] = acadoWorkspace.rk_S_traj[run * 42 + 8];
acadoWorkspace.rk_diffsPrev2[9] = acadoWorkspace.rk_S_traj[run * 42 + 9];
acadoWorkspace.rk_diffsPrev2[10] = acadoWorkspace.rk_S_traj[run * 42 + 10];
acadoWorkspace.rk_diffsPrev2[11] = acadoWorkspace.rk_S_traj[run * 42 + 11];
acadoWorkspace.rk_diffsPrev2[12] = acadoWorkspace.rk_S_traj[run * 42 + 12];
acadoWorkspace.rk_diffsPrev2[13] = acadoWorkspace.rk_S_traj[run * 42 + 13];
acadoWorkspace.rk_diffsPrev2[14] = acadoWorkspace.rk_S_traj[run * 42 + 14];
acadoWorkspace.rk_diffsPrev2[15] = acadoWorkspace.rk_S_traj[run * 42 + 15];
acadoWorkspace.rk_diffsPrev2[16] = acadoWorkspace.rk_S_traj[run * 42 + 16];
acadoWorkspace.rk_diffsPrev2[17] = acadoWorkspace.rk_S_traj[run * 42 + 17];
acadoWorkspace.rk_diffsPrev2[18] = acadoWorkspace.rk_S_traj[run * 42 + 18];
acadoWorkspace.rk_diffsPrev2[19] = acadoWorkspace.rk_S_traj[run * 42 + 19];
acadoWorkspace.rk_diffsPrev2[20] = acadoWorkspace.rk_S_traj[run * 42 + 20];
acadoWorkspace.rk_diffsPrev2[21] = acadoWorkspace.rk_S_traj[run * 42 + 21];
acadoWorkspace.rk_diffsPrev2[22] = acadoWorkspace.rk_S_traj[run * 42 + 22];
acadoWorkspace.rk_diffsPrev2[23] = acadoWorkspace.rk_S_traj[run * 42 + 23];
acadoWorkspace.rk_diffsPrev2[24] = acadoWorkspace.rk_S_traj[run * 42 + 24];
acadoWorkspace.rk_diffsPrev2[25] = acadoWorkspace.rk_S_traj[run * 42 + 25];
acadoWorkspace.rk_diffsPrev2[26] = acadoWorkspace.rk_S_traj[run * 42 + 26];
acadoWorkspace.rk_diffsPrev2[27] = acadoWorkspace.rk_S_traj[run * 42 + 27];
acadoWorkspace.rk_diffsPrev2[28] = acadoWorkspace.rk_S_traj[run * 42 + 28];
acadoWorkspace.rk_diffsPrev2[29] = acadoWorkspace.rk_S_traj[run * 42 + 29];
acadoWorkspace.rk_diffsPrev2[30] = acadoWorkspace.rk_S_traj[run * 42 + 30];
acadoWorkspace.rk_diffsPrev2[31] = acadoWorkspace.rk_S_traj[run * 42 + 31];
acadoWorkspace.rk_diffsPrev2[32] = acadoWorkspace.rk_S_traj[run * 42 + 32];
acadoWorkspace.rk_diffsPrev2[33] = acadoWorkspace.rk_S_traj[run * 42 + 33];
acadoWorkspace.rk_diffsPrev2[34] = acadoWorkspace.rk_S_traj[run * 42 + 34];
acadoWorkspace.rk_diffsPrev2[35] = acadoWorkspace.rk_S_traj[run * 42 + 35];
acadoWorkspace.rk_diffsPrev2[36] = acadoWorkspace.rk_S_traj[run * 42 + 36];
acadoWorkspace.rk_diffsPrev2[37] = acadoWorkspace.rk_S_traj[run * 42 + 37];
acadoWorkspace.rk_diffsPrev2[38] = acadoWorkspace.rk_S_traj[run * 42 + 38];
acadoWorkspace.rk_diffsPrev2[39] = acadoWorkspace.rk_S_traj[run * 42 + 39];
acadoWorkspace.rk_diffsPrev2[40] = acadoWorkspace.rk_S_traj[run * 42 + 40];
acadoWorkspace.rk_diffsPrev2[41] = acadoWorkspace.rk_S_traj[run * 42 + 41];
for (i = 0; i < 6; ++i)
{
tmp_index1 = ((run * 6) + (i)) * (7);
for (j = 0; j < 7; ++j)
{
tmp_index2 = (tmp_index1) + (j);
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[2]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2];
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[3]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1];
}
}
acadoWorkspace.rk_seed[6] = acadoWorkspace.rk_diffsPrev2[0];
acadoWorkspace.rk_seed[7] = acadoWorkspace.rk_diffsPrev2[1];
acadoWorkspace.rk_seed[8] = acadoWorkspace.rk_diffsPrev2[2];
acadoWorkspace.rk_seed[9] = acadoWorkspace.rk_diffsPrev2[3];
acadoWorkspace.rk_seed[10] = acadoWorkspace.rk_diffsPrev2[4];
acadoWorkspace.rk_seed[11] = acadoWorkspace.rk_diffsPrev2[5];
acadoWorkspace.rk_seed[12] = acadoWorkspace.rk_diffsPrev2[6];
acadoWorkspace.rk_seed[13] = acadoWorkspace.rk_diffsPrev2[7];
acadoWorkspace.rk_seed[14] = acadoWorkspace.rk_diffsPrev2[8];
acadoWorkspace.rk_seed[15] = acadoWorkspace.rk_diffsPrev2[9];
acadoWorkspace.rk_seed[16] = acadoWorkspace.rk_diffsPrev2[10];
acadoWorkspace.rk_seed[17] = acadoWorkspace.rk_diffsPrev2[11];
acadoWorkspace.rk_seed[18] = acadoWorkspace.rk_diffsPrev2[12];
acadoWorkspace.rk_seed[19] = acadoWorkspace.rk_diffsPrev2[13];
acadoWorkspace.rk_seed[20] = acadoWorkspace.rk_diffsPrev2[14];
acadoWorkspace.rk_seed[21] = acadoWorkspace.rk_diffsPrev2[15];
acadoWorkspace.rk_seed[22] = acadoWorkspace.rk_diffsPrev2[16];
acadoWorkspace.rk_seed[23] = acadoWorkspace.rk_diffsPrev2[17];
acadoWorkspace.rk_seed[24] = acadoWorkspace.rk_diffsPrev2[18];
acadoWorkspace.rk_seed[25] = acadoWorkspace.rk_diffsPrev2[19];
acadoWorkspace.rk_seed[26] = acadoWorkspace.rk_diffsPrev2[20];
acadoWorkspace.rk_seed[27] = acadoWorkspace.rk_diffsPrev2[21];
acadoWorkspace.rk_seed[28] = acadoWorkspace.rk_diffsPrev2[22];
acadoWorkspace.rk_seed[29] = acadoWorkspace.rk_diffsPrev2[23];
acadoWorkspace.rk_seed[30] = acadoWorkspace.rk_diffsPrev2[24];
acadoWorkspace.rk_seed[31] = acadoWorkspace.rk_diffsPrev2[25];
acadoWorkspace.rk_seed[32] = acadoWorkspace.rk_diffsPrev2[26];
acadoWorkspace.rk_seed[33] = acadoWorkspace.rk_diffsPrev2[27];
acadoWorkspace.rk_seed[34] = acadoWorkspace.rk_diffsPrev2[28];
acadoWorkspace.rk_seed[35] = acadoWorkspace.rk_diffsPrev2[29];
acadoWorkspace.rk_seed[36] = acadoWorkspace.rk_diffsPrev2[30];
acadoWorkspace.rk_seed[37] = acadoWorkspace.rk_diffsPrev2[31];
acadoWorkspace.rk_seed[38] = acadoWorkspace.rk_diffsPrev2[32];
acadoWorkspace.rk_seed[39] = acadoWorkspace.rk_diffsPrev2[33];
acadoWorkspace.rk_seed[40] = acadoWorkspace.rk_diffsPrev2[34];
acadoWorkspace.rk_seed[41] = acadoWorkspace.rk_diffsPrev2[35];
acadoWorkspace.rk_seed[42] = acadoWorkspace.rk_diffsPrev2[36];
acadoWorkspace.rk_seed[43] = acadoWorkspace.rk_diffsPrev2[37];
acadoWorkspace.rk_seed[44] = acadoWorkspace.rk_diffsPrev2[38];
acadoWorkspace.rk_seed[45] = acadoWorkspace.rk_diffsPrev2[39];
acadoWorkspace.rk_seed[46] = acadoWorkspace.rk_diffsPrev2[40];
acadoWorkspace.rk_seed[47] = acadoWorkspace.rk_diffsPrev2[41];
acadoWorkspace.rk_seed[48] = acadoWorkspace.rk_b_trans[6];
acadoWorkspace.rk_seed[49] = acadoWorkspace.rk_b_trans[7];
acadoWorkspace.rk_seed[50] = acadoWorkspace.rk_b_trans[8];
acadoWorkspace.rk_seed[51] = acadoWorkspace.rk_b_trans[9];
acadoWorkspace.rk_seed[52] = acadoWorkspace.rk_b_trans[10];
acadoWorkspace.rk_seed[53] = acadoWorkspace.rk_b_trans[11];
acado_acado_backward( acadoWorkspace.rk_seed, acadoWorkspace.rk_adjoint );
rk_eta[6] += acadoWorkspace.rk_adjoint[0];
rk_eta[7] += acadoWorkspace.rk_adjoint[1];
rk_eta[8] += acadoWorkspace.rk_adjoint[2];
rk_eta[9] += acadoWorkspace.rk_adjoint[3];
rk_eta[10] += acadoWorkspace.rk_adjoint[4];
rk_eta[11] += acadoWorkspace.rk_adjoint[5];
acadoWorkspace.rk_hess2[0] = acadoWorkspace.rk_adjoint[6];
acadoWorkspace.rk_hess2[1] = acadoWorkspace.rk_adjoint[7];
acadoWorkspace.rk_hess2[2] = acadoWorkspace.rk_adjoint[8];
acadoWorkspace.rk_hess2[3] = acadoWorkspace.rk_adjoint[9];
acadoWorkspace.rk_hess2[4] = acadoWorkspace.rk_adjoint[10];
acadoWorkspace.rk_hess2[5] = acadoWorkspace.rk_adjoint[11];
acadoWorkspace.rk_hess2[6] = acadoWorkspace.rk_adjoint[12];
acadoWorkspace.rk_hess2[7] = acadoWorkspace.rk_adjoint[13];
acadoWorkspace.rk_hess2[8] = acadoWorkspace.rk_adjoint[14];
acadoWorkspace.rk_hess2[9] = acadoWorkspace.rk_adjoint[15];
acadoWorkspace.rk_hess2[10] = acadoWorkspace.rk_adjoint[16];
acadoWorkspace.rk_hess2[11] = acadoWorkspace.rk_adjoint[17];
acadoWorkspace.rk_hess2[12] = acadoWorkspace.rk_adjoint[18];
acadoWorkspace.rk_hess2[13] = acadoWorkspace.rk_adjoint[19];
acadoWorkspace.rk_hess2[14] = acadoWorkspace.rk_adjoint[20];
acadoWorkspace.rk_hess2[15] = acadoWorkspace.rk_adjoint[21];
acadoWorkspace.rk_hess2[16] = acadoWorkspace.rk_adjoint[22];
acadoWorkspace.rk_hess2[17] = acadoWorkspace.rk_adjoint[23];
acadoWorkspace.rk_hess2[18] = acadoWorkspace.rk_adjoint[24];
acadoWorkspace.rk_hess2[19] = acadoWorkspace.rk_adjoint[25];
acadoWorkspace.rk_hess2[20] = acadoWorkspace.rk_adjoint[26];
acadoWorkspace.rk_hess2[21] = acadoWorkspace.rk_adjoint[27];
acadoWorkspace.rk_hess2[22] = acadoWorkspace.rk_adjoint[28];
acadoWorkspace.rk_hess2[23] = acadoWorkspace.rk_adjoint[29];
acadoWorkspace.rk_hess2[24] = acadoWorkspace.rk_adjoint[30];
acadoWorkspace.rk_hess2[25] = acadoWorkspace.rk_adjoint[31];
acadoWorkspace.rk_hess2[26] = acadoWorkspace.rk_adjoint[32];
acadoWorkspace.rk_hess2[27] = acadoWorkspace.rk_adjoint[33];
acadoWorkspace.rk_hess2[28] = acadoWorkspace.rk_adjoint[34];
acadoWorkspace.rk_hess2[29] = acadoWorkspace.rk_adjoint[35];
acadoWorkspace.rk_hess2[30] = acadoWorkspace.rk_adjoint[36];
acadoWorkspace.rk_hess2[31] = acadoWorkspace.rk_adjoint[37];
acadoWorkspace.rk_hess2[32] = acadoWorkspace.rk_adjoint[38];
acadoWorkspace.rk_hess2[33] = acadoWorkspace.rk_adjoint[39];
acadoWorkspace.rk_hess2[34] = acadoWorkspace.rk_adjoint[40];
acadoWorkspace.rk_hess2[35] = acadoWorkspace.rk_adjoint[41];
acadoWorkspace.rk_hess2[36] = acadoWorkspace.rk_adjoint[42];
acadoWorkspace.rk_hess2[37] = acadoWorkspace.rk_adjoint[43];
acadoWorkspace.rk_hess2[38] = acadoWorkspace.rk_adjoint[44];
acadoWorkspace.rk_hess2[39] = acadoWorkspace.rk_adjoint[45];
acadoWorkspace.rk_hess2[40] = acadoWorkspace.rk_adjoint[46];
acadoWorkspace.rk_hess2[41] = acadoWorkspace.rk_adjoint[47];
acadoWorkspace.rk_hess2[42] = acadoWorkspace.rk_adjoint[48];
acadoWorkspace.rk_hess2[43] = acadoWorkspace.rk_adjoint[49];
acadoWorkspace.rk_hess2[44] = acadoWorkspace.rk_adjoint[50];
acadoWorkspace.rk_hess2[45] = acadoWorkspace.rk_adjoint[51];
acadoWorkspace.rk_hess2[46] = acadoWorkspace.rk_adjoint[52];
acadoWorkspace.rk_hess2[47] = acadoWorkspace.rk_adjoint[53];
acadoWorkspace.rk_hess2[48] = acadoWorkspace.rk_adjoint[54];
acadoWorkspace.rk_diffsPrev2[0] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[8] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[9] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[10] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[11] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[12] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[13] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[14] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[15] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[16] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[17] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[18] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[19] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[20] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[21] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[22] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[23] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[24] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[25] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[26] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[27] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[28] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[29] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[30] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[31] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[32] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[33] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[34] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[35] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[36] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[37] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[38] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[39] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[40] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[41] = 0.0000000000000000e+00;
for (i = 0; i < 6; ++i)
{
tmp_index1 = ((run * 6) + (i)) * (7);
for (j = 0; j < 7; ++j)
{
tmp_index2 = (tmp_index1) + (j);
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[2]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2];
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[3]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1];
}
}
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i]*acadoWorkspace.rk_hess2[j];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 7]*acadoWorkspace.rk_hess2[j + 7];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 14]*acadoWorkspace.rk_hess2[j + 14];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 21]*acadoWorkspace.rk_hess2[j + 21];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 28]*acadoWorkspace.rk_hess2[j + 28];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 35]*acadoWorkspace.rk_hess2[j + 35];
}
}
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += acadoWorkspace.rk_hess2[(i * 7 + 42) + (j)];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 6]*acadoWorkspace.rk_hess2[j];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 13]*acadoWorkspace.rk_hess2[j + 7];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 20]*acadoWorkspace.rk_hess2[j + 14];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 27]*acadoWorkspace.rk_hess2[j + 21];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 34]*acadoWorkspace.rk_hess2[j + 28];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 41]*acadoWorkspace.rk_hess2[j + 35];
}
}
acadoWorkspace.rk_hess2[0] = rk_eta[54];
acadoWorkspace.rk_hess2[1] = rk_eta[55];
acadoWorkspace.rk_hess2[2] = rk_eta[56];
acadoWorkspace.rk_hess2[3] = rk_eta[57];
acadoWorkspace.rk_hess2[4] = rk_eta[58];
acadoWorkspace.rk_hess2[5] = rk_eta[59];
acadoWorkspace.rk_hess2[7] = rk_eta[60];
acadoWorkspace.rk_hess2[8] = rk_eta[61];
acadoWorkspace.rk_hess2[9] = rk_eta[62];
acadoWorkspace.rk_hess2[10] = rk_eta[63];
acadoWorkspace.rk_hess2[11] = rk_eta[64];
acadoWorkspace.rk_hess2[12] = rk_eta[65];
acadoWorkspace.rk_hess2[14] = rk_eta[66];
acadoWorkspace.rk_hess2[15] = rk_eta[67];
acadoWorkspace.rk_hess2[16] = rk_eta[68];
acadoWorkspace.rk_hess2[17] = rk_eta[69];
acadoWorkspace.rk_hess2[18] = rk_eta[70];
acadoWorkspace.rk_hess2[19] = rk_eta[71];
acadoWorkspace.rk_hess2[21] = rk_eta[72];
acadoWorkspace.rk_hess2[22] = rk_eta[73];
acadoWorkspace.rk_hess2[23] = rk_eta[74];
acadoWorkspace.rk_hess2[24] = rk_eta[75];
acadoWorkspace.rk_hess2[25] = rk_eta[76];
acadoWorkspace.rk_hess2[26] = rk_eta[77];
acadoWorkspace.rk_hess2[28] = rk_eta[78];
acadoWorkspace.rk_hess2[29] = rk_eta[79];
acadoWorkspace.rk_hess2[30] = rk_eta[80];
acadoWorkspace.rk_hess2[31] = rk_eta[81];
acadoWorkspace.rk_hess2[32] = rk_eta[82];
acadoWorkspace.rk_hess2[33] = rk_eta[83];
acadoWorkspace.rk_hess2[35] = rk_eta[84];
acadoWorkspace.rk_hess2[36] = rk_eta[85];
acadoWorkspace.rk_hess2[37] = rk_eta[86];
acadoWorkspace.rk_hess2[38] = rk_eta[87];
acadoWorkspace.rk_hess2[39] = rk_eta[88];
acadoWorkspace.rk_hess2[40] = rk_eta[89];
acadoWorkspace.rk_hess2[6] = rk_eta[90];
acadoWorkspace.rk_hess2[42] = acadoWorkspace.rk_hess2[6];
acadoWorkspace.rk_hess2[13] = rk_eta[91];
acadoWorkspace.rk_hess2[43] = acadoWorkspace.rk_hess2[13];
acadoWorkspace.rk_hess2[20] = rk_eta[92];
acadoWorkspace.rk_hess2[44] = acadoWorkspace.rk_hess2[20];
acadoWorkspace.rk_hess2[27] = rk_eta[93];
acadoWorkspace.rk_hess2[45] = acadoWorkspace.rk_hess2[27];
acadoWorkspace.rk_hess2[34] = rk_eta[94];
acadoWorkspace.rk_hess2[46] = acadoWorkspace.rk_hess2[34];
acadoWorkspace.rk_hess2[41] = rk_eta[95];
acadoWorkspace.rk_hess2[47] = acadoWorkspace.rk_hess2[41];
acadoWorkspace.rk_hess2[48] = rk_eta[96];
acadoWorkspace.rk_diffsPrev2[0] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[8] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[9] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[10] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[11] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[12] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[13] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[14] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[15] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[16] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[17] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[18] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[19] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[20] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[21] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[22] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[23] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[24] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[25] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[26] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[27] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[28] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[29] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[30] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[31] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[32] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[33] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[34] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[35] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[36] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[37] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[38] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[39] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[40] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[41] = 0.0000000000000000e+00;
for (i = 0; i < 6; ++i)
{
tmp_index1 = ((run * 6) + (i)) * (7);
for (j = 0; j < 7; ++j)
{
tmp_index2 = (tmp_index1) + (j);
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2]*(real_t)2.3999999999999999e-01 + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1]*(real_t)2.3999999999999999e-01;
}
}
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i]*acadoWorkspace.rk_hess2[j];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 7]*acadoWorkspace.rk_hess2[j + 7];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 14]*acadoWorkspace.rk_hess2[j + 14];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 21]*acadoWorkspace.rk_hess2[j + 21];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 28]*acadoWorkspace.rk_hess2[j + 28];
acadoWorkspace.rk_hess1[(i * 7) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 35]*acadoWorkspace.rk_hess2[j + 35];
}
}
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += acadoWorkspace.rk_hess2[(i * 7 + 42) + (j)];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 6]*acadoWorkspace.rk_hess2[j];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 13]*acadoWorkspace.rk_hess2[j + 7];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 20]*acadoWorkspace.rk_hess2[j + 14];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 27]*acadoWorkspace.rk_hess2[j + 21];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 34]*acadoWorkspace.rk_hess2[j + 28];
acadoWorkspace.rk_hess1[(i * 7 + 42) + (j)] += + acadoWorkspace.rk_diffsPrev2[i + 41]*acadoWorkspace.rk_hess2[j + 35];
}
}
rk_eta[54] = acadoWorkspace.rk_hess1[0];
rk_eta[55] = acadoWorkspace.rk_hess1[1];
rk_eta[56] = acadoWorkspace.rk_hess1[2];
rk_eta[57] = acadoWorkspace.rk_hess1[3];
rk_eta[58] = acadoWorkspace.rk_hess1[4];
rk_eta[59] = acadoWorkspace.rk_hess1[5];
rk_eta[60] = acadoWorkspace.rk_hess1[7];
rk_eta[61] = acadoWorkspace.rk_hess1[8];
rk_eta[62] = acadoWorkspace.rk_hess1[9];
rk_eta[63] = acadoWorkspace.rk_hess1[10];
rk_eta[64] = acadoWorkspace.rk_hess1[11];
rk_eta[65] = acadoWorkspace.rk_hess1[12];
rk_eta[66] = acadoWorkspace.rk_hess1[14];
rk_eta[67] = acadoWorkspace.rk_hess1[15];
rk_eta[68] = acadoWorkspace.rk_hess1[16];
rk_eta[69] = acadoWorkspace.rk_hess1[17];
rk_eta[70] = acadoWorkspace.rk_hess1[18];
rk_eta[71] = acadoWorkspace.rk_hess1[19];
rk_eta[72] = acadoWorkspace.rk_hess1[21];
rk_eta[73] = acadoWorkspace.rk_hess1[22];
rk_eta[74] = acadoWorkspace.rk_hess1[23];
rk_eta[75] = acadoWorkspace.rk_hess1[24];
rk_eta[76] = acadoWorkspace.rk_hess1[25];
rk_eta[77] = acadoWorkspace.rk_hess1[26];
rk_eta[78] = acadoWorkspace.rk_hess1[28];
rk_eta[79] = acadoWorkspace.rk_hess1[29];
rk_eta[80] = acadoWorkspace.rk_hess1[30];
rk_eta[81] = acadoWorkspace.rk_hess1[31];
rk_eta[82] = acadoWorkspace.rk_hess1[32];
rk_eta[83] = acadoWorkspace.rk_hess1[33];
rk_eta[84] = acadoWorkspace.rk_hess1[35];
rk_eta[85] = acadoWorkspace.rk_hess1[36];
rk_eta[86] = acadoWorkspace.rk_hess1[37];
rk_eta[87] = acadoWorkspace.rk_hess1[38];
rk_eta[88] = acadoWorkspace.rk_hess1[39];
rk_eta[89] = acadoWorkspace.rk_hess1[40];
rk_eta[90] = acadoWorkspace.rk_hess1[6];
rk_eta[91] = acadoWorkspace.rk_hess1[13];
rk_eta[92] = acadoWorkspace.rk_hess1[20];
rk_eta[93] = acadoWorkspace.rk_hess1[27];
rk_eta[94] = acadoWorkspace.rk_hess1[34];
rk_eta[95] = acadoWorkspace.rk_hess1[41];
rk_eta[96] = acadoWorkspace.rk_hess1[48];
acadoWorkspace.rk_ttt -= 2.0000000000000001e-01;
}
error = 0;
return error;
}



