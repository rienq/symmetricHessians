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




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 6 + 5];

acadoWorkspace.state[6] = acadoVariables.mu[lRun1 * 6];
acadoWorkspace.state[7] = acadoVariables.mu[lRun1 * 6 + 1];
acadoWorkspace.state[8] = acadoVariables.mu[lRun1 * 6 + 2];
acadoWorkspace.state[9] = acadoVariables.mu[lRun1 * 6 + 3];
acadoWorkspace.state[10] = acadoVariables.mu[lRun1 * 6 + 4];
acadoWorkspace.state[11] = acadoVariables.mu[lRun1 * 6 + 5];
acadoWorkspace.state[97] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 6] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 6 + 6];
acadoWorkspace.d[lRun1 * 6 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 6 + 7];
acadoWorkspace.d[lRun1 * 6 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 6 + 8];
acadoWorkspace.d[lRun1 * 6 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 6 + 9];
acadoWorkspace.d[lRun1 * 6 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 6 + 10];
acadoWorkspace.d[lRun1 * 6 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 6 + 11];

acadoWorkspace.evGx[lRun1 * 36] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 36 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 36 + 2] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 36 + 3] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 36 + 4] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 36 + 5] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 36 + 6] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 36 + 7] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 36 + 8] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 36 + 9] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 36 + 10] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 36 + 11] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 36 + 12] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 36 + 13] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 36 + 14] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 36 + 15] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 36 + 16] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 36 + 17] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 36 + 18] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 36 + 19] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 36 + 20] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 36 + 21] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 36 + 22] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 36 + 23] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 36 + 24] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 36 + 25] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 36 + 26] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 36 + 27] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 36 + 28] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 36 + 29] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 36 + 30] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 36 + 31] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 36 + 32] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 36 + 33] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 36 + 34] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 36 + 35] = acadoWorkspace.state[47];

acadoWorkspace.evGu[lRun1 * 6] = acadoWorkspace.state[48];
acadoWorkspace.evGu[lRun1 * 6 + 1] = acadoWorkspace.state[49];
acadoWorkspace.evGu[lRun1 * 6 + 2] = acadoWorkspace.state[50];
acadoWorkspace.evGu[lRun1 * 6 + 3] = acadoWorkspace.state[51];
acadoWorkspace.evGu[lRun1 * 6 + 4] = acadoWorkspace.state[52];
acadoWorkspace.evGu[lRun1 * 6 + 5] = acadoWorkspace.state[53];
acadoWorkspace.EH[lRun1 * 49] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[54];
acadoWorkspace.EH[lRun1 * 49 + 1] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[55];
acadoWorkspace.EH[lRun1 * 49 + 2] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[56];
acadoWorkspace.EH[lRun1 * 49 + 3] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[57];
acadoWorkspace.EH[lRun1 * 49 + 4] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[58];
acadoWorkspace.EH[lRun1 * 49 + 5] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[59];
acadoWorkspace.EH[lRun1 * 49 + 7] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[60];
acadoWorkspace.EH[lRun1 * 49 + 8] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[61];
acadoWorkspace.EH[lRun1 * 49 + 9] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[62];
acadoWorkspace.EH[lRun1 * 49 + 10] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[63];
acadoWorkspace.EH[lRun1 * 49 + 11] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[64];
acadoWorkspace.EH[lRun1 * 49 + 12] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[65];
acadoWorkspace.EH[lRun1 * 49 + 14] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[66];
acadoWorkspace.EH[lRun1 * 49 + 15] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[67];
acadoWorkspace.EH[lRun1 * 49 + 16] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[68];
acadoWorkspace.EH[lRun1 * 49 + 17] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[69];
acadoWorkspace.EH[lRun1 * 49 + 18] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[70];
acadoWorkspace.EH[lRun1 * 49 + 19] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[71];
acadoWorkspace.EH[lRun1 * 49 + 21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[72];
acadoWorkspace.EH[lRun1 * 49 + 22] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[73];
acadoWorkspace.EH[lRun1 * 49 + 23] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[74];
acadoWorkspace.EH[lRun1 * 49 + 24] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[75];
acadoWorkspace.EH[lRun1 * 49 + 25] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[76];
acadoWorkspace.EH[lRun1 * 49 + 26] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[77];
acadoWorkspace.EH[lRun1 * 49 + 28] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[78];
acadoWorkspace.EH[lRun1 * 49 + 29] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[79];
acadoWorkspace.EH[lRun1 * 49 + 30] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[80];
acadoWorkspace.EH[lRun1 * 49 + 31] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[81];
acadoWorkspace.EH[lRun1 * 49 + 32] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[82];
acadoWorkspace.EH[lRun1 * 49 + 33] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[83];
acadoWorkspace.EH[lRun1 * 49 + 35] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[84];
acadoWorkspace.EH[lRun1 * 49 + 36] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[85];
acadoWorkspace.EH[lRun1 * 49 + 37] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[86];
acadoWorkspace.EH[lRun1 * 49 + 38] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[87];
acadoWorkspace.EH[lRun1 * 49 + 39] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[88];
acadoWorkspace.EH[lRun1 * 49 + 40] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[89];
acadoWorkspace.EH[lRun1 * 49 + 42] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[90];
acadoWorkspace.EH[lRun1 * 49 + 6] = acadoWorkspace.EH[lRun1 * 49 + 42];
acadoWorkspace.EH[lRun1 * 49 + 43] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[91];
acadoWorkspace.EH[lRun1 * 49 + 13] = acadoWorkspace.EH[lRun1 * 49 + 43];
acadoWorkspace.EH[lRun1 * 49 + 44] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[92];
acadoWorkspace.EH[lRun1 * 49 + 20] = acadoWorkspace.EH[lRun1 * 49 + 44];
acadoWorkspace.EH[lRun1 * 49 + 45] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[93];
acadoWorkspace.EH[lRun1 * 49 + 27] = acadoWorkspace.EH[lRun1 * 49 + 45];
acadoWorkspace.EH[lRun1 * 49 + 46] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[94];
acadoWorkspace.EH[lRun1 * 49 + 34] = acadoWorkspace.EH[lRun1 * 49 + 46];
acadoWorkspace.EH[lRun1 * 49 + 47] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[95];
acadoWorkspace.EH[lRun1 * 49 + 41] = acadoWorkspace.EH[lRun1 * 49 + 47];
acadoWorkspace.EH[lRun1 * 49 + 48] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[96];
}
return ret;
}

void acado_evaluateMayer(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 27. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(-1.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = ((real_t)(0.0000000000000000e+00)-xd[5]);
out[1] = a[0];
out[2] = a[1];
out[3] = a[2];
out[4] = a[3];
out[5] = a[4];
out[6] = a[5];
out[7] = a[6];
out[8] = a[7];
out[9] = a[8];
out[10] = a[9];
out[11] = a[10];
out[12] = a[11];
out[13] = a[7];
out[14] = a[12];
out[15] = a[13];
out[16] = a[14];
out[17] = a[15];
out[18] = a[16];
out[19] = a[8];
out[20] = a[13];
out[21] = a[17];
out[22] = a[18];
out[23] = a[19];
out[24] = a[20];
out[25] = a[9];
out[26] = a[14];
out[27] = a[18];
out[28] = a[21];
out[29] = a[22];
out[30] = a[23];
out[31] = a[10];
out[32] = a[15];
out[33] = a[19];
out[34] = a[22];
out[35] = a[24];
out[36] = a[25];
out[37] = a[11];
out[38] = a[16];
out[39] = a[20];
out[40] = a[23];
out[41] = a[25];
out[42] = a[26];
}

void acado_addObjEndTerm( real_t* const tmpFxxEnd, real_t* const tmpEH_N )
{
tmpEH_N[0] = tmpFxxEnd[0];
tmpEH_N[1] = tmpFxxEnd[1];
tmpEH_N[2] = tmpFxxEnd[2];
tmpEH_N[3] = tmpFxxEnd[3];
tmpEH_N[4] = tmpFxxEnd[4];
tmpEH_N[5] = tmpFxxEnd[5];
tmpEH_N[6] = tmpFxxEnd[6];
tmpEH_N[7] = tmpFxxEnd[7];
tmpEH_N[8] = tmpFxxEnd[8];
tmpEH_N[9] = tmpFxxEnd[9];
tmpEH_N[10] = tmpFxxEnd[10];
tmpEH_N[11] = tmpFxxEnd[11];
tmpEH_N[12] = tmpFxxEnd[12];
tmpEH_N[13] = tmpFxxEnd[13];
tmpEH_N[14] = tmpFxxEnd[14];
tmpEH_N[15] = tmpFxxEnd[15];
tmpEH_N[16] = tmpFxxEnd[16];
tmpEH_N[17] = tmpFxxEnd[17];
tmpEH_N[18] = tmpFxxEnd[18];
tmpEH_N[19] = tmpFxxEnd[19];
tmpEH_N[20] = tmpFxxEnd[20];
tmpEH_N[21] = tmpFxxEnd[21];
tmpEH_N[22] = tmpFxxEnd[22];
tmpEH_N[23] = tmpFxxEnd[23];
tmpEH_N[24] = tmpFxxEnd[24];
tmpEH_N[25] = tmpFxxEnd[25];
tmpEH_N[26] = tmpFxxEnd[26];
tmpEH_N[27] = tmpFxxEnd[27];
tmpEH_N[28] = tmpFxxEnd[28];
tmpEH_N[29] = tmpFxxEnd[29];
tmpEH_N[30] = tmpFxxEnd[30];
tmpEH_N[31] = tmpFxxEnd[31];
tmpEH_N[32] = tmpFxxEnd[32];
tmpEH_N[33] = tmpFxxEnd[33];
tmpEH_N[34] = tmpFxxEnd[34];
tmpEH_N[35] = tmpFxxEnd[35];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.g[runObj] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 6] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 6 + 1] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 6 + 2] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 6 + 3] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 6 + 4] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 6 + 5] = 0.0000000000000000e+00;
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjEndTerm( &(acadoWorkspace.objValueOut[ 7 ]), acadoWorkspace.EH_N );
acadoWorkspace.QDy[120] = acadoWorkspace.objValueOut[1];
acadoWorkspace.QDy[121] = acadoWorkspace.objValueOut[2];
acadoWorkspace.QDy[122] = acadoWorkspace.objValueOut[3];
acadoWorkspace.QDy[123] = acadoWorkspace.objValueOut[4];
acadoWorkspace.QDy[124] = acadoWorkspace.objValueOut[5];
acadoWorkspace.QDy[125] = acadoWorkspace.objValueOut[6];

}

void acado_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acado_regularize( &(acadoWorkspace.EH[ lRun1 * 49 ]) );
acadoWorkspace.Q1[lRun1 * 36] = acadoWorkspace.EH[lRun1 * 49];
acadoWorkspace.Q1[lRun1 * 36 + 1] = acadoWorkspace.EH[lRun1 * 49 + 1];
acadoWorkspace.Q1[lRun1 * 36 + 2] = acadoWorkspace.EH[lRun1 * 49 + 2];
acadoWorkspace.Q1[lRun1 * 36 + 3] = acadoWorkspace.EH[lRun1 * 49 + 3];
acadoWorkspace.Q1[lRun1 * 36 + 4] = acadoWorkspace.EH[lRun1 * 49 + 4];
acadoWorkspace.Q1[lRun1 * 36 + 5] = acadoWorkspace.EH[lRun1 * 49 + 5];
acadoWorkspace.Q1[lRun1 * 36 + 6] = acadoWorkspace.EH[lRun1 * 49 + 7];
acadoWorkspace.Q1[lRun1 * 36 + 7] = acadoWorkspace.EH[lRun1 * 49 + 8];
acadoWorkspace.Q1[lRun1 * 36 + 8] = acadoWorkspace.EH[lRun1 * 49 + 9];
acadoWorkspace.Q1[lRun1 * 36 + 9] = acadoWorkspace.EH[lRun1 * 49 + 10];
acadoWorkspace.Q1[lRun1 * 36 + 10] = acadoWorkspace.EH[lRun1 * 49 + 11];
acadoWorkspace.Q1[lRun1 * 36 + 11] = acadoWorkspace.EH[lRun1 * 49 + 12];
acadoWorkspace.Q1[lRun1 * 36 + 12] = acadoWorkspace.EH[lRun1 * 49 + 14];
acadoWorkspace.Q1[lRun1 * 36 + 13] = acadoWorkspace.EH[lRun1 * 49 + 15];
acadoWorkspace.Q1[lRun1 * 36 + 14] = acadoWorkspace.EH[lRun1 * 49 + 16];
acadoWorkspace.Q1[lRun1 * 36 + 15] = acadoWorkspace.EH[lRun1 * 49 + 17];
acadoWorkspace.Q1[lRun1 * 36 + 16] = acadoWorkspace.EH[lRun1 * 49 + 18];
acadoWorkspace.Q1[lRun1 * 36 + 17] = acadoWorkspace.EH[lRun1 * 49 + 19];
acadoWorkspace.Q1[lRun1 * 36 + 18] = acadoWorkspace.EH[lRun1 * 49 + 21];
acadoWorkspace.Q1[lRun1 * 36 + 19] = acadoWorkspace.EH[lRun1 * 49 + 22];
acadoWorkspace.Q1[lRun1 * 36 + 20] = acadoWorkspace.EH[lRun1 * 49 + 23];
acadoWorkspace.Q1[lRun1 * 36 + 21] = acadoWorkspace.EH[lRun1 * 49 + 24];
acadoWorkspace.Q1[lRun1 * 36 + 22] = acadoWorkspace.EH[lRun1 * 49 + 25];
acadoWorkspace.Q1[lRun1 * 36 + 23] = acadoWorkspace.EH[lRun1 * 49 + 26];
acadoWorkspace.Q1[lRun1 * 36 + 24] = acadoWorkspace.EH[lRun1 * 49 + 28];
acadoWorkspace.Q1[lRun1 * 36 + 25] = acadoWorkspace.EH[lRun1 * 49 + 29];
acadoWorkspace.Q1[lRun1 * 36 + 26] = acadoWorkspace.EH[lRun1 * 49 + 30];
acadoWorkspace.Q1[lRun1 * 36 + 27] = acadoWorkspace.EH[lRun1 * 49 + 31];
acadoWorkspace.Q1[lRun1 * 36 + 28] = acadoWorkspace.EH[lRun1 * 49 + 32];
acadoWorkspace.Q1[lRun1 * 36 + 29] = acadoWorkspace.EH[lRun1 * 49 + 33];
acadoWorkspace.Q1[lRun1 * 36 + 30] = acadoWorkspace.EH[lRun1 * 49 + 35];
acadoWorkspace.Q1[lRun1 * 36 + 31] = acadoWorkspace.EH[lRun1 * 49 + 36];
acadoWorkspace.Q1[lRun1 * 36 + 32] = acadoWorkspace.EH[lRun1 * 49 + 37];
acadoWorkspace.Q1[lRun1 * 36 + 33] = acadoWorkspace.EH[lRun1 * 49 + 38];
acadoWorkspace.Q1[lRun1 * 36 + 34] = acadoWorkspace.EH[lRun1 * 49 + 39];
acadoWorkspace.Q1[lRun1 * 36 + 35] = acadoWorkspace.EH[lRun1 * 49 + 40];
acadoWorkspace.S1[lRun1 * 6] = acadoWorkspace.EH[lRun1 * 49 + 6];
acadoWorkspace.S1[lRun1 * 6 + 1] = acadoWorkspace.EH[lRun1 * 49 + 13];
acadoWorkspace.S1[lRun1 * 6 + 2] = acadoWorkspace.EH[lRun1 * 49 + 20];
acadoWorkspace.S1[lRun1 * 6 + 3] = acadoWorkspace.EH[lRun1 * 49 + 27];
acadoWorkspace.S1[lRun1 * 6 + 4] = acadoWorkspace.EH[lRun1 * 49 + 34];
acadoWorkspace.S1[lRun1 * 6 + 5] = acadoWorkspace.EH[lRun1 * 49 + 41];
acadoWorkspace.R1[lRun1] = acadoWorkspace.EH[lRun1 * 49 + 48];
}
acadoWorkspace.QN1[0] = acadoWorkspace.EH_N[0];
acadoWorkspace.QN1[1] = acadoWorkspace.EH_N[1];
acadoWorkspace.QN1[2] = acadoWorkspace.EH_N[2];
acadoWorkspace.QN1[3] = acadoWorkspace.EH_N[3];
acadoWorkspace.QN1[4] = acadoWorkspace.EH_N[4];
acadoWorkspace.QN1[5] = acadoWorkspace.EH_N[5];
acadoWorkspace.QN1[6] = acadoWorkspace.EH_N[6];
acadoWorkspace.QN1[7] = acadoWorkspace.EH_N[7];
acadoWorkspace.QN1[8] = acadoWorkspace.EH_N[8];
acadoWorkspace.QN1[9] = acadoWorkspace.EH_N[9];
acadoWorkspace.QN1[10] = acadoWorkspace.EH_N[10];
acadoWorkspace.QN1[11] = acadoWorkspace.EH_N[11];
acadoWorkspace.QN1[12] = acadoWorkspace.EH_N[12];
acadoWorkspace.QN1[13] = acadoWorkspace.EH_N[13];
acadoWorkspace.QN1[14] = acadoWorkspace.EH_N[14];
acadoWorkspace.QN1[15] = acadoWorkspace.EH_N[15];
acadoWorkspace.QN1[16] = acadoWorkspace.EH_N[16];
acadoWorkspace.QN1[17] = acadoWorkspace.EH_N[17];
acadoWorkspace.QN1[18] = acadoWorkspace.EH_N[18];
acadoWorkspace.QN1[19] = acadoWorkspace.EH_N[19];
acadoWorkspace.QN1[20] = acadoWorkspace.EH_N[20];
acadoWorkspace.QN1[21] = acadoWorkspace.EH_N[21];
acadoWorkspace.QN1[22] = acadoWorkspace.EH_N[22];
acadoWorkspace.QN1[23] = acadoWorkspace.EH_N[23];
acadoWorkspace.QN1[24] = acadoWorkspace.EH_N[24];
acadoWorkspace.QN1[25] = acadoWorkspace.EH_N[25];
acadoWorkspace.QN1[26] = acadoWorkspace.EH_N[26];
acadoWorkspace.QN1[27] = acadoWorkspace.EH_N[27];
acadoWorkspace.QN1[28] = acadoWorkspace.EH_N[28];
acadoWorkspace.QN1[29] = acadoWorkspace.EH_N[29];
acadoWorkspace.QN1[30] = acadoWorkspace.EH_N[30];
acadoWorkspace.QN1[31] = acadoWorkspace.EH_N[31];
acadoWorkspace.QN1[32] = acadoWorkspace.EH_N[32];
acadoWorkspace.QN1[33] = acadoWorkspace.EH_N[33];
acadoWorkspace.QN1[34] = acadoWorkspace.EH_N[34];
acadoWorkspace.QN1[35] = acadoWorkspace.EH_N[35];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2] + Gx1[3]*Gu1[3] + Gx1[4]*Gu1[4] + Gx1[5]*Gu1[5];
Gu2[1] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[8]*Gu1[2] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[5];
Gu2[2] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[1] + Gx1[14]*Gu1[2] + Gx1[15]*Gu1[3] + Gx1[16]*Gu1[4] + Gx1[17]*Gu1[5];
Gu2[3] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[1] + Gx1[20]*Gu1[2] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[5];
Gu2[4] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[1] + Gx1[26]*Gu1[2] + Gx1[27]*Gu1[3] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[5];
Gu2[5] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[1] + Gx1[32]*Gu1[2] + Gx1[33]*Gu1[3] + Gx1[34]*Gu1[4] + Gx1[35]*Gu1[5];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3] + Gu1[4]*Gu2[4] + Gu1[5]*Gu2[5];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] += + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3] + Gu1[4]*Gu2[4] + Gu1[5]*Gu2[5];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 21] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3] + Gu1[4]*Gu2[4] + Gu1[5]*Gu2[5] + R11[0];
acadoWorkspace.H[iRow * 21] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[6]*Gu1[1] + Gx1[12]*Gu1[2] + Gx1[18]*Gu1[3] + Gx1[24]*Gu1[4] + Gx1[30]*Gu1[5];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[13]*Gu1[2] + Gx1[19]*Gu1[3] + Gx1[25]*Gu1[4] + Gx1[31]*Gu1[5];
Gu2[2] = + Gx1[2]*Gu1[0] + Gx1[8]*Gu1[1] + Gx1[14]*Gu1[2] + Gx1[20]*Gu1[3] + Gx1[26]*Gu1[4] + Gx1[32]*Gu1[5];
Gu2[3] = + Gx1[3]*Gu1[0] + Gx1[9]*Gu1[1] + Gx1[15]*Gu1[2] + Gx1[21]*Gu1[3] + Gx1[27]*Gu1[4] + Gx1[33]*Gu1[5];
Gu2[4] = + Gx1[4]*Gu1[0] + Gx1[10]*Gu1[1] + Gx1[16]*Gu1[2] + Gx1[22]*Gu1[3] + Gx1[28]*Gu1[4] + Gx1[34]*Gu1[5];
Gu2[5] = + Gx1[5]*Gu1[0] + Gx1[11]*Gu1[1] + Gx1[17]*Gu1[2] + Gx1[23]*Gu1[3] + Gx1[29]*Gu1[4] + Gx1[35]*Gu1[5];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[1] + Q11[2]*Gu1[2] + Q11[3]*Gu1[3] + Q11[4]*Gu1[4] + Q11[5]*Gu1[5] + Gu2[0];
Gu3[1] = + Q11[6]*Gu1[0] + Q11[7]*Gu1[1] + Q11[8]*Gu1[2] + Q11[9]*Gu1[3] + Q11[10]*Gu1[4] + Q11[11]*Gu1[5] + Gu2[1];
Gu3[2] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[1] + Q11[14]*Gu1[2] + Q11[15]*Gu1[3] + Q11[16]*Gu1[4] + Q11[17]*Gu1[5] + Gu2[2];
Gu3[3] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[1] + Q11[20]*Gu1[2] + Q11[21]*Gu1[3] + Q11[22]*Gu1[4] + Q11[23]*Gu1[5] + Gu2[3];
Gu3[4] = + Q11[24]*Gu1[0] + Q11[25]*Gu1[1] + Q11[26]*Gu1[2] + Q11[27]*Gu1[3] + Q11[28]*Gu1[4] + Q11[29]*Gu1[5] + Gu2[4];
Gu3[5] = + Q11[30]*Gu1[0] + Q11[31]*Gu1[1] + Q11[32]*Gu1[2] + Q11[33]*Gu1[3] + Q11[34]*Gu1[4] + Q11[35]*Gu1[5] + Gu2[5];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[6]*w11[1] + Gx1[12]*w11[2] + Gx1[18]*w11[3] + Gx1[24]*w11[4] + Gx1[30]*w11[5] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[7]*w11[1] + Gx1[13]*w11[2] + Gx1[19]*w11[3] + Gx1[25]*w11[4] + Gx1[31]*w11[5] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[8]*w11[1] + Gx1[14]*w11[2] + Gx1[20]*w11[3] + Gx1[26]*w11[4] + Gx1[32]*w11[5] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[9]*w11[1] + Gx1[15]*w11[2] + Gx1[21]*w11[3] + Gx1[27]*w11[4] + Gx1[33]*w11[5] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[10]*w11[1] + Gx1[16]*w11[2] + Gx1[22]*w11[3] + Gx1[28]*w11[4] + Gx1[34]*w11[5] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[11]*w11[1] + Gx1[17]*w11[2] + Gx1[23]*w11[3] + Gx1[29]*w11[4] + Gx1[35]*w11[5] + w12[5];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1] + Gu1[2]*w11[2] + Gu1[3]*w11[3] + Gu1[4]*w11[4] + Gu1[5]*w11[5];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1] + Gu1[2]*w11[2] + Gu1[3]*w11[3] + Gu1[4]*w11[4] + Gu1[5]*w11[5];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + w12[0];
w13[1] = + Q11[6]*w11[0] + Q11[7]*w11[1] + Q11[8]*w11[2] + Q11[9]*w11[3] + Q11[10]*w11[4] + Q11[11]*w11[5] + w12[1];
w13[2] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + Q11[16]*w11[4] + Q11[17]*w11[5] + w12[2];
w13[3] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + w12[3];
w13[4] = + Q11[24]*w11[0] + Q11[25]*w11[1] + Q11[26]*w11[2] + Q11[27]*w11[3] + Q11[28]*w11[4] + Q11[29]*w11[5] + w12[4];
w13[5] = + Q11[30]*w11[0] + Q11[31]*w11[1] + Q11[32]*w11[2] + Q11[33]*w11[3] + Q11[34]*w11[4] + Q11[35]*w11[5] + w12[5];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5];
w12[1] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2] + Gx1[9]*w11[3] + Gx1[10]*w11[4] + Gx1[11]*w11[5];
w12[2] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5];
w12[3] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5];
w12[4] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5];
w12[5] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5];
w12[1] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2] + Gx1[9]*w11[3] + Gx1[10]*w11[4] + Gx1[11]*w11[5];
w12[2] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5];
w12[3] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5];
w12[4] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5];
w12[5] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5];
w12[0] += + Gu1[0]*U1[0];
w12[1] += + Gu1[1]*U1[0];
w12[2] += + Gu1[2]*U1[0];
w12[3] += + Gu1[3]*U1[0];
w12[4] += + Gu1[4]*U1[0];
w12[5] += + Gu1[5]*U1[0];
}

void acado_expansionStep2( real_t* const QDy1, real_t* const Q11, real_t* const w11, real_t* const Gu1, real_t* const U1, real_t* const Gx1, real_t* const mu1, real_t* const mu2 )
{
mu1[0] += QDy1[0];
mu1[1] += QDy1[1];
mu1[2] += QDy1[2];
mu1[3] += QDy1[3];
mu1[4] += QDy1[4];
mu1[5] += QDy1[5];
mu1[0] += + w11[0]*Q11[0] + w11[1]*Q11[1] + w11[2]*Q11[2] + w11[3]*Q11[3] + w11[4]*Q11[4] + w11[5]*Q11[5];
mu1[1] += + w11[0]*Q11[6] + w11[1]*Q11[7] + w11[2]*Q11[8] + w11[3]*Q11[9] + w11[4]*Q11[10] + w11[5]*Q11[11];
mu1[2] += + w11[0]*Q11[12] + w11[1]*Q11[13] + w11[2]*Q11[14] + w11[3]*Q11[15] + w11[4]*Q11[16] + w11[5]*Q11[17];
mu1[3] += + w11[0]*Q11[18] + w11[1]*Q11[19] + w11[2]*Q11[20] + w11[3]*Q11[21] + w11[4]*Q11[22] + w11[5]*Q11[23];
mu1[4] += + w11[0]*Q11[24] + w11[1]*Q11[25] + w11[2]*Q11[26] + w11[3]*Q11[27] + w11[4]*Q11[28] + w11[5]*Q11[29];
mu1[5] += + w11[0]*Q11[30] + w11[1]*Q11[31] + w11[2]*Q11[32] + w11[3]*Q11[33] + w11[4]*Q11[34] + w11[5]*Q11[35];
mu1[0] += + U1[0]*Gu1[0];
mu1[1] += + U1[0]*Gu1[1];
mu1[2] += + U1[0]*Gu1[2];
mu1[3] += + U1[0]*Gu1[3];
mu1[4] += + U1[0]*Gu1[4];
mu1[5] += + U1[0]*Gu1[5];
mu1[0] += + mu2[0]*Gx1[0] + mu2[1]*Gx1[6] + mu2[2]*Gx1[12] + mu2[3]*Gx1[18] + mu2[4]*Gx1[24] + mu2[5]*Gx1[30];
mu1[1] += + mu2[0]*Gx1[1] + mu2[1]*Gx1[7] + mu2[2]*Gx1[13] + mu2[3]*Gx1[19] + mu2[4]*Gx1[25] + mu2[5]*Gx1[31];
mu1[2] += + mu2[0]*Gx1[2] + mu2[1]*Gx1[8] + mu2[2]*Gx1[14] + mu2[3]*Gx1[20] + mu2[4]*Gx1[26] + mu2[5]*Gx1[32];
mu1[3] += + mu2[0]*Gx1[3] + mu2[1]*Gx1[9] + mu2[2]*Gx1[15] + mu2[3]*Gx1[21] + mu2[4]*Gx1[27] + mu2[5]*Gx1[33];
mu1[4] += + mu2[0]*Gx1[4] + mu2[1]*Gx1[10] + mu2[2]*Gx1[16] + mu2[3]*Gx1[22] + mu2[4]*Gx1[28] + mu2[5]*Gx1[34];
mu1[5] += + mu2[0]*Gx1[5] + mu2[1]*Gx1[11] + mu2[2]*Gx1[17] + mu2[3]*Gx1[23] + mu2[4]*Gx1[29] + mu2[5]*Gx1[35];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] = acadoWorkspace.H[(iCol * 20) + (iRow)];
}

void acado_multRDy( real_t* const RDy1 )
{
}

void acado_multQDy( real_t* const QDy1 )
{
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 6 */
static const int xBoundIndices[ 6 ] = 
{ 120, 121, 122, 123, 124, 125 };
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 6 ]), &(acadoWorkspace.E[ lRun3 * 6 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (6)) * (6)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (6)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (6)) * (1)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (6)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 6 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 6 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (6)) * (1)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 36 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (6)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 ]), &(acadoWorkspace.evGu[ lRun2 * 6 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

acadoWorkspace.sbar[6] = acadoWorkspace.d[0];
acadoWorkspace.sbar[7] = acadoWorkspace.d[1];
acadoWorkspace.sbar[8] = acadoWorkspace.d[2];
acadoWorkspace.sbar[9] = acadoWorkspace.d[3];
acadoWorkspace.sbar[10] = acadoWorkspace.d[4];
acadoWorkspace.sbar[11] = acadoWorkspace.d[5];
acadoWorkspace.sbar[12] = acadoWorkspace.d[6];
acadoWorkspace.sbar[13] = acadoWorkspace.d[7];
acadoWorkspace.sbar[14] = acadoWorkspace.d[8];
acadoWorkspace.sbar[15] = acadoWorkspace.d[9];
acadoWorkspace.sbar[16] = acadoWorkspace.d[10];
acadoWorkspace.sbar[17] = acadoWorkspace.d[11];
acadoWorkspace.sbar[18] = acadoWorkspace.d[12];
acadoWorkspace.sbar[19] = acadoWorkspace.d[13];
acadoWorkspace.sbar[20] = acadoWorkspace.d[14];
acadoWorkspace.sbar[21] = acadoWorkspace.d[15];
acadoWorkspace.sbar[22] = acadoWorkspace.d[16];
acadoWorkspace.sbar[23] = acadoWorkspace.d[17];
acadoWorkspace.sbar[24] = acadoWorkspace.d[18];
acadoWorkspace.sbar[25] = acadoWorkspace.d[19];
acadoWorkspace.sbar[26] = acadoWorkspace.d[20];
acadoWorkspace.sbar[27] = acadoWorkspace.d[21];
acadoWorkspace.sbar[28] = acadoWorkspace.d[22];
acadoWorkspace.sbar[29] = acadoWorkspace.d[23];
acadoWorkspace.sbar[30] = acadoWorkspace.d[24];
acadoWorkspace.sbar[31] = acadoWorkspace.d[25];
acadoWorkspace.sbar[32] = acadoWorkspace.d[26];
acadoWorkspace.sbar[33] = acadoWorkspace.d[27];
acadoWorkspace.sbar[34] = acadoWorkspace.d[28];
acadoWorkspace.sbar[35] = acadoWorkspace.d[29];
acadoWorkspace.sbar[36] = acadoWorkspace.d[30];
acadoWorkspace.sbar[37] = acadoWorkspace.d[31];
acadoWorkspace.sbar[38] = acadoWorkspace.d[32];
acadoWorkspace.sbar[39] = acadoWorkspace.d[33];
acadoWorkspace.sbar[40] = acadoWorkspace.d[34];
acadoWorkspace.sbar[41] = acadoWorkspace.d[35];
acadoWorkspace.sbar[42] = acadoWorkspace.d[36];
acadoWorkspace.sbar[43] = acadoWorkspace.d[37];
acadoWorkspace.sbar[44] = acadoWorkspace.d[38];
acadoWorkspace.sbar[45] = acadoWorkspace.d[39];
acadoWorkspace.sbar[46] = acadoWorkspace.d[40];
acadoWorkspace.sbar[47] = acadoWorkspace.d[41];
acadoWorkspace.sbar[48] = acadoWorkspace.d[42];
acadoWorkspace.sbar[49] = acadoWorkspace.d[43];
acadoWorkspace.sbar[50] = acadoWorkspace.d[44];
acadoWorkspace.sbar[51] = acadoWorkspace.d[45];
acadoWorkspace.sbar[52] = acadoWorkspace.d[46];
acadoWorkspace.sbar[53] = acadoWorkspace.d[47];
acadoWorkspace.sbar[54] = acadoWorkspace.d[48];
acadoWorkspace.sbar[55] = acadoWorkspace.d[49];
acadoWorkspace.sbar[56] = acadoWorkspace.d[50];
acadoWorkspace.sbar[57] = acadoWorkspace.d[51];
acadoWorkspace.sbar[58] = acadoWorkspace.d[52];
acadoWorkspace.sbar[59] = acadoWorkspace.d[53];
acadoWorkspace.sbar[60] = acadoWorkspace.d[54];
acadoWorkspace.sbar[61] = acadoWorkspace.d[55];
acadoWorkspace.sbar[62] = acadoWorkspace.d[56];
acadoWorkspace.sbar[63] = acadoWorkspace.d[57];
acadoWorkspace.sbar[64] = acadoWorkspace.d[58];
acadoWorkspace.sbar[65] = acadoWorkspace.d[59];
acadoWorkspace.sbar[66] = acadoWorkspace.d[60];
acadoWorkspace.sbar[67] = acadoWorkspace.d[61];
acadoWorkspace.sbar[68] = acadoWorkspace.d[62];
acadoWorkspace.sbar[69] = acadoWorkspace.d[63];
acadoWorkspace.sbar[70] = acadoWorkspace.d[64];
acadoWorkspace.sbar[71] = acadoWorkspace.d[65];
acadoWorkspace.sbar[72] = acadoWorkspace.d[66];
acadoWorkspace.sbar[73] = acadoWorkspace.d[67];
acadoWorkspace.sbar[74] = acadoWorkspace.d[68];
acadoWorkspace.sbar[75] = acadoWorkspace.d[69];
acadoWorkspace.sbar[76] = acadoWorkspace.d[70];
acadoWorkspace.sbar[77] = acadoWorkspace.d[71];
acadoWorkspace.sbar[78] = acadoWorkspace.d[72];
acadoWorkspace.sbar[79] = acadoWorkspace.d[73];
acadoWorkspace.sbar[80] = acadoWorkspace.d[74];
acadoWorkspace.sbar[81] = acadoWorkspace.d[75];
acadoWorkspace.sbar[82] = acadoWorkspace.d[76];
acadoWorkspace.sbar[83] = acadoWorkspace.d[77];
acadoWorkspace.sbar[84] = acadoWorkspace.d[78];
acadoWorkspace.sbar[85] = acadoWorkspace.d[79];
acadoWorkspace.sbar[86] = acadoWorkspace.d[80];
acadoWorkspace.sbar[87] = acadoWorkspace.d[81];
acadoWorkspace.sbar[88] = acadoWorkspace.d[82];
acadoWorkspace.sbar[89] = acadoWorkspace.d[83];
acadoWorkspace.sbar[90] = acadoWorkspace.d[84];
acadoWorkspace.sbar[91] = acadoWorkspace.d[85];
acadoWorkspace.sbar[92] = acadoWorkspace.d[86];
acadoWorkspace.sbar[93] = acadoWorkspace.d[87];
acadoWorkspace.sbar[94] = acadoWorkspace.d[88];
acadoWorkspace.sbar[95] = acadoWorkspace.d[89];
acadoWorkspace.sbar[96] = acadoWorkspace.d[90];
acadoWorkspace.sbar[97] = acadoWorkspace.d[91];
acadoWorkspace.sbar[98] = acadoWorkspace.d[92];
acadoWorkspace.sbar[99] = acadoWorkspace.d[93];
acadoWorkspace.sbar[100] = acadoWorkspace.d[94];
acadoWorkspace.sbar[101] = acadoWorkspace.d[95];
acadoWorkspace.sbar[102] = acadoWorkspace.d[96];
acadoWorkspace.sbar[103] = acadoWorkspace.d[97];
acadoWorkspace.sbar[104] = acadoWorkspace.d[98];
acadoWorkspace.sbar[105] = acadoWorkspace.d[99];
acadoWorkspace.sbar[106] = acadoWorkspace.d[100];
acadoWorkspace.sbar[107] = acadoWorkspace.d[101];
acadoWorkspace.sbar[108] = acadoWorkspace.d[102];
acadoWorkspace.sbar[109] = acadoWorkspace.d[103];
acadoWorkspace.sbar[110] = acadoWorkspace.d[104];
acadoWorkspace.sbar[111] = acadoWorkspace.d[105];
acadoWorkspace.sbar[112] = acadoWorkspace.d[106];
acadoWorkspace.sbar[113] = acadoWorkspace.d[107];
acadoWorkspace.sbar[114] = acadoWorkspace.d[108];
acadoWorkspace.sbar[115] = acadoWorkspace.d[109];
acadoWorkspace.sbar[116] = acadoWorkspace.d[110];
acadoWorkspace.sbar[117] = acadoWorkspace.d[111];
acadoWorkspace.sbar[118] = acadoWorkspace.d[112];
acadoWorkspace.sbar[119] = acadoWorkspace.d[113];
acadoWorkspace.sbar[120] = acadoWorkspace.d[114];
acadoWorkspace.sbar[121] = acadoWorkspace.d[115];
acadoWorkspace.sbar[122] = acadoWorkspace.d[116];
acadoWorkspace.sbar[123] = acadoWorkspace.d[117];
acadoWorkspace.sbar[124] = acadoWorkspace.d[118];
acadoWorkspace.sbar[125] = acadoWorkspace.d[119];

for (lRun1 = 0; lRun1 < 6; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 6;
lRun4 = ((lRun3) / (6)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 39)) / (2)) + (lRun4)) - (1)) * (6)) + ((lRun3) % (6));
acadoWorkspace.A[(lRun1 * 20) + (lRun2)] = acadoWorkspace.E[lRun5];
}
}

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 120 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[120];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[121];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[122];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[123];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[124];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[125];
acado_macBTw1( &(acadoWorkspace.evGu[ 114 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 19 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 114 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.g[ 19 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 684 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 684 ]), &(acadoWorkspace.sbar[ 114 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 102 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 17 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 102 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.g[ 17 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 612 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 612 ]), &(acadoWorkspace.sbar[ 102 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 90 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 540 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 504 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 78 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 13 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 78 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 468 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 66 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 11 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 66 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 66 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 42 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 42 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 18 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];

tmp = acadoWorkspace.sbar[120] + acadoVariables.x[120];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = acadoWorkspace.sbar[121] + acadoVariables.x[121];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = acadoWorkspace.sbar[122] + acadoVariables.x[122];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = acadoWorkspace.sbar[123] + acadoVariables.x[123];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = acadoWorkspace.sbar[124] + acadoVariables.x[124];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = acadoWorkspace.sbar[125] + acadoVariables.x[125];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.d[0];
acadoWorkspace.sbar[7] = acadoWorkspace.d[1];
acadoWorkspace.sbar[8] = acadoWorkspace.d[2];
acadoWorkspace.sbar[9] = acadoWorkspace.d[3];
acadoWorkspace.sbar[10] = acadoWorkspace.d[4];
acadoWorkspace.sbar[11] = acadoWorkspace.d[5];
acadoWorkspace.sbar[12] = acadoWorkspace.d[6];
acadoWorkspace.sbar[13] = acadoWorkspace.d[7];
acadoWorkspace.sbar[14] = acadoWorkspace.d[8];
acadoWorkspace.sbar[15] = acadoWorkspace.d[9];
acadoWorkspace.sbar[16] = acadoWorkspace.d[10];
acadoWorkspace.sbar[17] = acadoWorkspace.d[11];
acadoWorkspace.sbar[18] = acadoWorkspace.d[12];
acadoWorkspace.sbar[19] = acadoWorkspace.d[13];
acadoWorkspace.sbar[20] = acadoWorkspace.d[14];
acadoWorkspace.sbar[21] = acadoWorkspace.d[15];
acadoWorkspace.sbar[22] = acadoWorkspace.d[16];
acadoWorkspace.sbar[23] = acadoWorkspace.d[17];
acadoWorkspace.sbar[24] = acadoWorkspace.d[18];
acadoWorkspace.sbar[25] = acadoWorkspace.d[19];
acadoWorkspace.sbar[26] = acadoWorkspace.d[20];
acadoWorkspace.sbar[27] = acadoWorkspace.d[21];
acadoWorkspace.sbar[28] = acadoWorkspace.d[22];
acadoWorkspace.sbar[29] = acadoWorkspace.d[23];
acadoWorkspace.sbar[30] = acadoWorkspace.d[24];
acadoWorkspace.sbar[31] = acadoWorkspace.d[25];
acadoWorkspace.sbar[32] = acadoWorkspace.d[26];
acadoWorkspace.sbar[33] = acadoWorkspace.d[27];
acadoWorkspace.sbar[34] = acadoWorkspace.d[28];
acadoWorkspace.sbar[35] = acadoWorkspace.d[29];
acadoWorkspace.sbar[36] = acadoWorkspace.d[30];
acadoWorkspace.sbar[37] = acadoWorkspace.d[31];
acadoWorkspace.sbar[38] = acadoWorkspace.d[32];
acadoWorkspace.sbar[39] = acadoWorkspace.d[33];
acadoWorkspace.sbar[40] = acadoWorkspace.d[34];
acadoWorkspace.sbar[41] = acadoWorkspace.d[35];
acadoWorkspace.sbar[42] = acadoWorkspace.d[36];
acadoWorkspace.sbar[43] = acadoWorkspace.d[37];
acadoWorkspace.sbar[44] = acadoWorkspace.d[38];
acadoWorkspace.sbar[45] = acadoWorkspace.d[39];
acadoWorkspace.sbar[46] = acadoWorkspace.d[40];
acadoWorkspace.sbar[47] = acadoWorkspace.d[41];
acadoWorkspace.sbar[48] = acadoWorkspace.d[42];
acadoWorkspace.sbar[49] = acadoWorkspace.d[43];
acadoWorkspace.sbar[50] = acadoWorkspace.d[44];
acadoWorkspace.sbar[51] = acadoWorkspace.d[45];
acadoWorkspace.sbar[52] = acadoWorkspace.d[46];
acadoWorkspace.sbar[53] = acadoWorkspace.d[47];
acadoWorkspace.sbar[54] = acadoWorkspace.d[48];
acadoWorkspace.sbar[55] = acadoWorkspace.d[49];
acadoWorkspace.sbar[56] = acadoWorkspace.d[50];
acadoWorkspace.sbar[57] = acadoWorkspace.d[51];
acadoWorkspace.sbar[58] = acadoWorkspace.d[52];
acadoWorkspace.sbar[59] = acadoWorkspace.d[53];
acadoWorkspace.sbar[60] = acadoWorkspace.d[54];
acadoWorkspace.sbar[61] = acadoWorkspace.d[55];
acadoWorkspace.sbar[62] = acadoWorkspace.d[56];
acadoWorkspace.sbar[63] = acadoWorkspace.d[57];
acadoWorkspace.sbar[64] = acadoWorkspace.d[58];
acadoWorkspace.sbar[65] = acadoWorkspace.d[59];
acadoWorkspace.sbar[66] = acadoWorkspace.d[60];
acadoWorkspace.sbar[67] = acadoWorkspace.d[61];
acadoWorkspace.sbar[68] = acadoWorkspace.d[62];
acadoWorkspace.sbar[69] = acadoWorkspace.d[63];
acadoWorkspace.sbar[70] = acadoWorkspace.d[64];
acadoWorkspace.sbar[71] = acadoWorkspace.d[65];
acadoWorkspace.sbar[72] = acadoWorkspace.d[66];
acadoWorkspace.sbar[73] = acadoWorkspace.d[67];
acadoWorkspace.sbar[74] = acadoWorkspace.d[68];
acadoWorkspace.sbar[75] = acadoWorkspace.d[69];
acadoWorkspace.sbar[76] = acadoWorkspace.d[70];
acadoWorkspace.sbar[77] = acadoWorkspace.d[71];
acadoWorkspace.sbar[78] = acadoWorkspace.d[72];
acadoWorkspace.sbar[79] = acadoWorkspace.d[73];
acadoWorkspace.sbar[80] = acadoWorkspace.d[74];
acadoWorkspace.sbar[81] = acadoWorkspace.d[75];
acadoWorkspace.sbar[82] = acadoWorkspace.d[76];
acadoWorkspace.sbar[83] = acadoWorkspace.d[77];
acadoWorkspace.sbar[84] = acadoWorkspace.d[78];
acadoWorkspace.sbar[85] = acadoWorkspace.d[79];
acadoWorkspace.sbar[86] = acadoWorkspace.d[80];
acadoWorkspace.sbar[87] = acadoWorkspace.d[81];
acadoWorkspace.sbar[88] = acadoWorkspace.d[82];
acadoWorkspace.sbar[89] = acadoWorkspace.d[83];
acadoWorkspace.sbar[90] = acadoWorkspace.d[84];
acadoWorkspace.sbar[91] = acadoWorkspace.d[85];
acadoWorkspace.sbar[92] = acadoWorkspace.d[86];
acadoWorkspace.sbar[93] = acadoWorkspace.d[87];
acadoWorkspace.sbar[94] = acadoWorkspace.d[88];
acadoWorkspace.sbar[95] = acadoWorkspace.d[89];
acadoWorkspace.sbar[96] = acadoWorkspace.d[90];
acadoWorkspace.sbar[97] = acadoWorkspace.d[91];
acadoWorkspace.sbar[98] = acadoWorkspace.d[92];
acadoWorkspace.sbar[99] = acadoWorkspace.d[93];
acadoWorkspace.sbar[100] = acadoWorkspace.d[94];
acadoWorkspace.sbar[101] = acadoWorkspace.d[95];
acadoWorkspace.sbar[102] = acadoWorkspace.d[96];
acadoWorkspace.sbar[103] = acadoWorkspace.d[97];
acadoWorkspace.sbar[104] = acadoWorkspace.d[98];
acadoWorkspace.sbar[105] = acadoWorkspace.d[99];
acadoWorkspace.sbar[106] = acadoWorkspace.d[100];
acadoWorkspace.sbar[107] = acadoWorkspace.d[101];
acadoWorkspace.sbar[108] = acadoWorkspace.d[102];
acadoWorkspace.sbar[109] = acadoWorkspace.d[103];
acadoWorkspace.sbar[110] = acadoWorkspace.d[104];
acadoWorkspace.sbar[111] = acadoWorkspace.d[105];
acadoWorkspace.sbar[112] = acadoWorkspace.d[106];
acadoWorkspace.sbar[113] = acadoWorkspace.d[107];
acadoWorkspace.sbar[114] = acadoWorkspace.d[108];
acadoWorkspace.sbar[115] = acadoWorkspace.d[109];
acadoWorkspace.sbar[116] = acadoWorkspace.d[110];
acadoWorkspace.sbar[117] = acadoWorkspace.d[111];
acadoWorkspace.sbar[118] = acadoWorkspace.d[112];
acadoWorkspace.sbar[119] = acadoWorkspace.d[113];
acadoWorkspace.sbar[120] = acadoWorkspace.d[114];
acadoWorkspace.sbar[121] = acadoWorkspace.d[115];
acadoWorkspace.sbar[122] = acadoWorkspace.d[116];
acadoWorkspace.sbar[123] = acadoWorkspace.d[117];
acadoWorkspace.sbar[124] = acadoWorkspace.d[118];
acadoWorkspace.sbar[125] = acadoWorkspace.d[119];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGu[ 66 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.evGu[ 78 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.evGu[ 102 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.evGu[ 114 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 120 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
acadoVariables.x[55] += acadoWorkspace.sbar[55];
acadoVariables.x[56] += acadoWorkspace.sbar[56];
acadoVariables.x[57] += acadoWorkspace.sbar[57];
acadoVariables.x[58] += acadoWorkspace.sbar[58];
acadoVariables.x[59] += acadoWorkspace.sbar[59];
acadoVariables.x[60] += acadoWorkspace.sbar[60];
acadoVariables.x[61] += acadoWorkspace.sbar[61];
acadoVariables.x[62] += acadoWorkspace.sbar[62];
acadoVariables.x[63] += acadoWorkspace.sbar[63];
acadoVariables.x[64] += acadoWorkspace.sbar[64];
acadoVariables.x[65] += acadoWorkspace.sbar[65];
acadoVariables.x[66] += acadoWorkspace.sbar[66];
acadoVariables.x[67] += acadoWorkspace.sbar[67];
acadoVariables.x[68] += acadoWorkspace.sbar[68];
acadoVariables.x[69] += acadoWorkspace.sbar[69];
acadoVariables.x[70] += acadoWorkspace.sbar[70];
acadoVariables.x[71] += acadoWorkspace.sbar[71];
acadoVariables.x[72] += acadoWorkspace.sbar[72];
acadoVariables.x[73] += acadoWorkspace.sbar[73];
acadoVariables.x[74] += acadoWorkspace.sbar[74];
acadoVariables.x[75] += acadoWorkspace.sbar[75];
acadoVariables.x[76] += acadoWorkspace.sbar[76];
acadoVariables.x[77] += acadoWorkspace.sbar[77];
acadoVariables.x[78] += acadoWorkspace.sbar[78];
acadoVariables.x[79] += acadoWorkspace.sbar[79];
acadoVariables.x[80] += acadoWorkspace.sbar[80];
acadoVariables.x[81] += acadoWorkspace.sbar[81];
acadoVariables.x[82] += acadoWorkspace.sbar[82];
acadoVariables.x[83] += acadoWorkspace.sbar[83];
acadoVariables.x[84] += acadoWorkspace.sbar[84];
acadoVariables.x[85] += acadoWorkspace.sbar[85];
acadoVariables.x[86] += acadoWorkspace.sbar[86];
acadoVariables.x[87] += acadoWorkspace.sbar[87];
acadoVariables.x[88] += acadoWorkspace.sbar[88];
acadoVariables.x[89] += acadoWorkspace.sbar[89];
acadoVariables.x[90] += acadoWorkspace.sbar[90];
acadoVariables.x[91] += acadoWorkspace.sbar[91];
acadoVariables.x[92] += acadoWorkspace.sbar[92];
acadoVariables.x[93] += acadoWorkspace.sbar[93];
acadoVariables.x[94] += acadoWorkspace.sbar[94];
acadoVariables.x[95] += acadoWorkspace.sbar[95];
acadoVariables.x[96] += acadoWorkspace.sbar[96];
acadoVariables.x[97] += acadoWorkspace.sbar[97];
acadoVariables.x[98] += acadoWorkspace.sbar[98];
acadoVariables.x[99] += acadoWorkspace.sbar[99];
acadoVariables.x[100] += acadoWorkspace.sbar[100];
acadoVariables.x[101] += acadoWorkspace.sbar[101];
acadoVariables.x[102] += acadoWorkspace.sbar[102];
acadoVariables.x[103] += acadoWorkspace.sbar[103];
acadoVariables.x[104] += acadoWorkspace.sbar[104];
acadoVariables.x[105] += acadoWorkspace.sbar[105];
acadoVariables.x[106] += acadoWorkspace.sbar[106];
acadoVariables.x[107] += acadoWorkspace.sbar[107];
acadoVariables.x[108] += acadoWorkspace.sbar[108];
acadoVariables.x[109] += acadoWorkspace.sbar[109];
acadoVariables.x[110] += acadoWorkspace.sbar[110];
acadoVariables.x[111] += acadoWorkspace.sbar[111];
acadoVariables.x[112] += acadoWorkspace.sbar[112];
acadoVariables.x[113] += acadoWorkspace.sbar[113];
acadoVariables.x[114] += acadoWorkspace.sbar[114];
acadoVariables.x[115] += acadoWorkspace.sbar[115];
acadoVariables.x[116] += acadoWorkspace.sbar[116];
acadoVariables.x[117] += acadoWorkspace.sbar[117];
acadoVariables.x[118] += acadoWorkspace.sbar[118];
acadoVariables.x[119] += acadoWorkspace.sbar[119];
acadoVariables.x[120] += acadoWorkspace.sbar[120];
acadoVariables.x[121] += acadoWorkspace.sbar[121];
acadoVariables.x[122] += acadoWorkspace.sbar[122];
acadoVariables.x[123] += acadoWorkspace.sbar[123];
acadoVariables.x[124] += acadoWorkspace.sbar[124];
acadoVariables.x[125] += acadoWorkspace.sbar[125];
acadoVariables.mu[114] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[20];
acadoVariables.mu[115] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[21];
acadoVariables.mu[116] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[22];
acadoVariables.mu[117] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[23];
acadoVariables.mu[118] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[24];
acadoVariables.mu[119] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[25];
acadoVariables.mu[114] += + acadoWorkspace.sbar[120]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[121]*acadoWorkspace.QN1[6] + acadoWorkspace.sbar[122]*acadoWorkspace.QN1[12] + acadoWorkspace.sbar[123]*acadoWorkspace.QN1[18] + acadoWorkspace.sbar[124]*acadoWorkspace.QN1[24] + acadoWorkspace.sbar[125]*acadoWorkspace.QN1[30];
acadoVariables.mu[115] += + acadoWorkspace.sbar[120]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[121]*acadoWorkspace.QN1[7] + acadoWorkspace.sbar[122]*acadoWorkspace.QN1[13] + acadoWorkspace.sbar[123]*acadoWorkspace.QN1[19] + acadoWorkspace.sbar[124]*acadoWorkspace.QN1[25] + acadoWorkspace.sbar[125]*acadoWorkspace.QN1[31];
acadoVariables.mu[116] += + acadoWorkspace.sbar[120]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[121]*acadoWorkspace.QN1[8] + acadoWorkspace.sbar[122]*acadoWorkspace.QN1[14] + acadoWorkspace.sbar[123]*acadoWorkspace.QN1[20] + acadoWorkspace.sbar[124]*acadoWorkspace.QN1[26] + acadoWorkspace.sbar[125]*acadoWorkspace.QN1[32];
acadoVariables.mu[117] += + acadoWorkspace.sbar[120]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[121]*acadoWorkspace.QN1[9] + acadoWorkspace.sbar[122]*acadoWorkspace.QN1[15] + acadoWorkspace.sbar[123]*acadoWorkspace.QN1[21] + acadoWorkspace.sbar[124]*acadoWorkspace.QN1[27] + acadoWorkspace.sbar[125]*acadoWorkspace.QN1[33];
acadoVariables.mu[118] += + acadoWorkspace.sbar[120]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[121]*acadoWorkspace.QN1[10] + acadoWorkspace.sbar[122]*acadoWorkspace.QN1[16] + acadoWorkspace.sbar[123]*acadoWorkspace.QN1[22] + acadoWorkspace.sbar[124]*acadoWorkspace.QN1[28] + acadoWorkspace.sbar[125]*acadoWorkspace.QN1[34];
acadoVariables.mu[119] += + acadoWorkspace.sbar[120]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[121]*acadoWorkspace.QN1[11] + acadoWorkspace.sbar[122]*acadoWorkspace.QN1[17] + acadoWorkspace.sbar[123]*acadoWorkspace.QN1[23] + acadoWorkspace.sbar[124]*acadoWorkspace.QN1[29] + acadoWorkspace.sbar[125]*acadoWorkspace.QN1[35];
acadoVariables.mu[114] += acadoWorkspace.QDy[120];
acadoVariables.mu[115] += acadoWorkspace.QDy[121];
acadoVariables.mu[116] += acadoWorkspace.QDy[122];
acadoVariables.mu[117] += acadoWorkspace.QDy[123];
acadoVariables.mu[118] += acadoWorkspace.QDy[124];
acadoVariables.mu[119] += acadoWorkspace.QDy[125];
acadoVariables.mu[108] = 0.0000000000000000e+00;
acadoVariables.mu[109] = 0.0000000000000000e+00;
acadoVariables.mu[110] = 0.0000000000000000e+00;
acadoVariables.mu[111] = 0.0000000000000000e+00;
acadoVariables.mu[112] = 0.0000000000000000e+00;
acadoVariables.mu[113] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.Q1[ 684 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.S1[ 114 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoVariables.mu[ 108 ]), &(acadoVariables.mu[ 114 ]) );
acadoVariables.mu[102] = 0.0000000000000000e+00;
acadoVariables.mu[103] = 0.0000000000000000e+00;
acadoVariables.mu[104] = 0.0000000000000000e+00;
acadoVariables.mu[105] = 0.0000000000000000e+00;
acadoVariables.mu[106] = 0.0000000000000000e+00;
acadoVariables.mu[107] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoVariables.mu[ 102 ]), &(acadoVariables.mu[ 108 ]) );
acadoVariables.mu[96] = 0.0000000000000000e+00;
acadoVariables.mu[97] = 0.0000000000000000e+00;
acadoVariables.mu[98] = 0.0000000000000000e+00;
acadoVariables.mu[99] = 0.0000000000000000e+00;
acadoVariables.mu[100] = 0.0000000000000000e+00;
acadoVariables.mu[101] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.Q1[ 612 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.S1[ 102 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoVariables.mu[ 96 ]), &(acadoVariables.mu[ 102 ]) );
acadoVariables.mu[90] = 0.0000000000000000e+00;
acadoVariables.mu[91] = 0.0000000000000000e+00;
acadoVariables.mu[92] = 0.0000000000000000e+00;
acadoVariables.mu[93] = 0.0000000000000000e+00;
acadoVariables.mu[94] = 0.0000000000000000e+00;
acadoVariables.mu[95] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoVariables.mu[ 90 ]), &(acadoVariables.mu[ 96 ]) );
acadoVariables.mu[84] = 0.0000000000000000e+00;
acadoVariables.mu[85] = 0.0000000000000000e+00;
acadoVariables.mu[86] = 0.0000000000000000e+00;
acadoVariables.mu[87] = 0.0000000000000000e+00;
acadoVariables.mu[88] = 0.0000000000000000e+00;
acadoVariables.mu[89] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.Q1[ 540 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.S1[ 90 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoVariables.mu[ 84 ]), &(acadoVariables.mu[ 90 ]) );
acadoVariables.mu[78] = 0.0000000000000000e+00;
acadoVariables.mu[79] = 0.0000000000000000e+00;
acadoVariables.mu[80] = 0.0000000000000000e+00;
acadoVariables.mu[81] = 0.0000000000000000e+00;
acadoVariables.mu[82] = 0.0000000000000000e+00;
acadoVariables.mu[83] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.Q1[ 504 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoVariables.mu[ 78 ]), &(acadoVariables.mu[ 84 ]) );
acadoVariables.mu[72] = 0.0000000000000000e+00;
acadoVariables.mu[73] = 0.0000000000000000e+00;
acadoVariables.mu[74] = 0.0000000000000000e+00;
acadoVariables.mu[75] = 0.0000000000000000e+00;
acadoVariables.mu[76] = 0.0000000000000000e+00;
acadoVariables.mu[77] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.Q1[ 468 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.S1[ 78 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoVariables.mu[ 72 ]), &(acadoVariables.mu[ 78 ]) );
acadoVariables.mu[66] = 0.0000000000000000e+00;
acadoVariables.mu[67] = 0.0000000000000000e+00;
acadoVariables.mu[68] = 0.0000000000000000e+00;
acadoVariables.mu[69] = 0.0000000000000000e+00;
acadoVariables.mu[70] = 0.0000000000000000e+00;
acadoVariables.mu[71] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoVariables.mu[ 66 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[60] = 0.0000000000000000e+00;
acadoVariables.mu[61] = 0.0000000000000000e+00;
acadoVariables.mu[62] = 0.0000000000000000e+00;
acadoVariables.mu[63] = 0.0000000000000000e+00;
acadoVariables.mu[64] = 0.0000000000000000e+00;
acadoVariables.mu[65] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.S1[ 66 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoVariables.mu[ 60 ]), &(acadoVariables.mu[ 66 ]) );
acadoVariables.mu[54] = 0.0000000000000000e+00;
acadoVariables.mu[55] = 0.0000000000000000e+00;
acadoVariables.mu[56] = 0.0000000000000000e+00;
acadoVariables.mu[57] = 0.0000000000000000e+00;
acadoVariables.mu[58] = 0.0000000000000000e+00;
acadoVariables.mu[59] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoVariables.mu[ 54 ]), &(acadoVariables.mu[ 60 ]) );
acadoVariables.mu[48] = 0.0000000000000000e+00;
acadoVariables.mu[49] = 0.0000000000000000e+00;
acadoVariables.mu[50] = 0.0000000000000000e+00;
acadoVariables.mu[51] = 0.0000000000000000e+00;
acadoVariables.mu[52] = 0.0000000000000000e+00;
acadoVariables.mu[53] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoVariables.mu[ 48 ]), &(acadoVariables.mu[ 54 ]) );
acadoVariables.mu[42] = 0.0000000000000000e+00;
acadoVariables.mu[43] = 0.0000000000000000e+00;
acadoVariables.mu[44] = 0.0000000000000000e+00;
acadoVariables.mu[45] = 0.0000000000000000e+00;
acadoVariables.mu[46] = 0.0000000000000000e+00;
acadoVariables.mu[47] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoVariables.mu[ 42 ]), &(acadoVariables.mu[ 48 ]) );
acadoVariables.mu[36] = 0.0000000000000000e+00;
acadoVariables.mu[37] = 0.0000000000000000e+00;
acadoVariables.mu[38] = 0.0000000000000000e+00;
acadoVariables.mu[39] = 0.0000000000000000e+00;
acadoVariables.mu[40] = 0.0000000000000000e+00;
acadoVariables.mu[41] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.S1[ 42 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoVariables.mu[ 36 ]), &(acadoVariables.mu[ 42 ]) );
acadoVariables.mu[30] = 0.0000000000000000e+00;
acadoVariables.mu[31] = 0.0000000000000000e+00;
acadoVariables.mu[32] = 0.0000000000000000e+00;
acadoVariables.mu[33] = 0.0000000000000000e+00;
acadoVariables.mu[34] = 0.0000000000000000e+00;
acadoVariables.mu[35] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoVariables.mu[ 30 ]), &(acadoVariables.mu[ 36 ]) );
acadoVariables.mu[24] = 0.0000000000000000e+00;
acadoVariables.mu[25] = 0.0000000000000000e+00;
acadoVariables.mu[26] = 0.0000000000000000e+00;
acadoVariables.mu[27] = 0.0000000000000000e+00;
acadoVariables.mu[28] = 0.0000000000000000e+00;
acadoVariables.mu[29] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.S1[ 30 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoVariables.mu[ 24 ]), &(acadoVariables.mu[ 30 ]) );
acadoVariables.mu[18] = 0.0000000000000000e+00;
acadoVariables.mu[19] = 0.0000000000000000e+00;
acadoVariables.mu[20] = 0.0000000000000000e+00;
acadoVariables.mu[21] = 0.0000000000000000e+00;
acadoVariables.mu[22] = 0.0000000000000000e+00;
acadoVariables.mu[23] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoVariables.mu[ 18 ]), &(acadoVariables.mu[ 24 ]) );
acadoVariables.mu[12] = 0.0000000000000000e+00;
acadoVariables.mu[13] = 0.0000000000000000e+00;
acadoVariables.mu[14] = 0.0000000000000000e+00;
acadoVariables.mu[15] = 0.0000000000000000e+00;
acadoVariables.mu[16] = 0.0000000000000000e+00;
acadoVariables.mu[17] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.S1[ 18 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoVariables.mu[ 12 ]), &(acadoVariables.mu[ 18 ]) );
acadoVariables.mu[6] = 0.0000000000000000e+00;
acadoVariables.mu[7] = 0.0000000000000000e+00;
acadoVariables.mu[8] = 0.0000000000000000e+00;
acadoVariables.mu[9] = 0.0000000000000000e+00;
acadoVariables.mu[10] = 0.0000000000000000e+00;
acadoVariables.mu[11] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.S1[ 12 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoVariables.mu[ 6 ]), &(acadoVariables.mu[ 12 ]) );
acadoVariables.mu[0] = 0.0000000000000000e+00;
acadoVariables.mu[1] = 0.0000000000000000e+00;
acadoVariables.mu[2] = 0.0000000000000000e+00;
acadoVariables.mu[3] = 0.0000000000000000e+00;
acadoVariables.mu[4] = 0.0000000000000000e+00;
acadoVariables.mu[5] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.S1[ 6 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.evGx[ 36 ]), acadoVariables.mu, &(acadoVariables.mu[ 6 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_regularizeHessian(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = 2.8699999999999999e+01;
acadoVariables.lbValues[1] = 2.8699999999999999e+01;
acadoVariables.lbValues[2] = 2.8699999999999999e+01;
acadoVariables.lbValues[3] = 2.8699999999999999e+01;
acadoVariables.lbValues[4] = 2.8699999999999999e+01;
acadoVariables.lbValues[5] = 2.8699999999999999e+01;
acadoVariables.lbValues[6] = 2.8699999999999999e+01;
acadoVariables.lbValues[7] = 2.8699999999999999e+01;
acadoVariables.lbValues[8] = 2.8699999999999999e+01;
acadoVariables.lbValues[9] = 2.8699999999999999e+01;
acadoVariables.lbValues[10] = 2.8699999999999999e+01;
acadoVariables.lbValues[11] = 2.8699999999999999e+01;
acadoVariables.lbValues[12] = 2.8699999999999999e+01;
acadoVariables.lbValues[13] = 2.8699999999999999e+01;
acadoVariables.lbValues[14] = 2.8699999999999999e+01;
acadoVariables.lbValues[15] = 2.8699999999999999e+01;
acadoVariables.lbValues[16] = 2.8699999999999999e+01;
acadoVariables.lbValues[17] = 2.8699999999999999e+01;
acadoVariables.lbValues[18] = 2.8699999999999999e+01;
acadoVariables.lbValues[19] = 2.8699999999999999e+01;
acadoVariables.ubValues[0] = 4.0000000000000000e+01;
acadoVariables.ubValues[1] = 4.0000000000000000e+01;
acadoVariables.ubValues[2] = 4.0000000000000000e+01;
acadoVariables.ubValues[3] = 4.0000000000000000e+01;
acadoVariables.ubValues[4] = 4.0000000000000000e+01;
acadoVariables.ubValues[5] = 4.0000000000000000e+01;
acadoVariables.ubValues[6] = 4.0000000000000000e+01;
acadoVariables.ubValues[7] = 4.0000000000000000e+01;
acadoVariables.ubValues[8] = 4.0000000000000000e+01;
acadoVariables.ubValues[9] = 4.0000000000000000e+01;
acadoVariables.ubValues[10] = 4.0000000000000000e+01;
acadoVariables.ubValues[11] = 4.0000000000000000e+01;
acadoVariables.ubValues[12] = 4.0000000000000000e+01;
acadoVariables.ubValues[13] = 4.0000000000000000e+01;
acadoVariables.ubValues[14] = 4.0000000000000000e+01;
acadoVariables.ubValues[15] = 4.0000000000000000e+01;
acadoVariables.ubValues[16] = 4.0000000000000000e+01;
acadoVariables.ubValues[17] = 4.0000000000000000e+01;
acadoVariables.ubValues[18] = 4.0000000000000000e+01;
acadoVariables.ubValues[19] = 4.0000000000000000e+01;
{ int lCopy; for (lCopy = 0; lCopy < 6; lCopy++) acadoVariables.lbAValues[ lCopy ] = 0; }
acadoVariables.ubAValues[0] = 0.0000000000000000e+00;
acadoVariables.ubAValues[1] = 0.0000000000000000e+00;
acadoVariables.ubAValues[2] = 0.0000000000000000e+00;
acadoVariables.ubAValues[3] = 2.7839999999999998e+02;
acadoVariables.ubAValues[4] = 1.5696000000000001e+03;
acadoVariables.ubAValues[5] = 1.0000000000000000e+08;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 6];
acadoWorkspace.state[1] = acadoVariables.x[index * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 6 + 5];
acadoWorkspace.state[97] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 6 + 6] = acadoWorkspace.state[0];
acadoVariables.x[index * 6 + 7] = acadoWorkspace.state[1];
acadoVariables.x[index * 6 + 8] = acadoWorkspace.state[2];
acadoVariables.x[index * 6 + 9] = acadoWorkspace.state[3];
acadoVariables.x[index * 6 + 10] = acadoWorkspace.state[4];
acadoVariables.x[index * 6 + 11] = acadoWorkspace.state[5];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 6] = acadoVariables.x[index * 6 + 6];
acadoVariables.x[index * 6 + 1] = acadoVariables.x[index * 6 + 7];
acadoVariables.x[index * 6 + 2] = acadoVariables.x[index * 6 + 8];
acadoVariables.x[index * 6 + 3] = acadoVariables.x[index * 6 + 9];
acadoVariables.x[index * 6 + 4] = acadoVariables.x[index * 6 + 10];
acadoVariables.x[index * 6 + 5] = acadoVariables.x[index * 6 + 11];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
acadoVariables.x[123] = xEnd[3];
acadoVariables.x[124] = xEnd[4];
acadoVariables.x[125] = xEnd[5];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[120];
acadoWorkspace.state[1] = acadoVariables.x[121];
acadoWorkspace.state[2] = acadoVariables.x[122];
acadoWorkspace.state[3] = acadoVariables.x[123];
acadoWorkspace.state[4] = acadoVariables.x[124];
acadoWorkspace.state[5] = acadoVariables.x[125];
if (uEnd != 0)
{
acadoWorkspace.state[97] = uEnd[0];
}
else
{
acadoWorkspace.state[97] = acadoVariables.u[19];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[120] = acadoWorkspace.state[0];
acadoVariables.x[121] = acadoWorkspace.state[1];
acadoVariables.x[122] = acadoWorkspace.state[2];
acadoVariables.x[123] = acadoWorkspace.state[3];
acadoVariables.x[124] = acadoWorkspace.state[4];
acadoVariables.x[125] = acadoWorkspace.state[5];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[19] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 6; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
objVal = 0.0000000000000000e+00;
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
return objVal;
}

