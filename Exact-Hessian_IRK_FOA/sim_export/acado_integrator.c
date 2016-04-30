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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;
/* Vector of auxiliary variables; number of elements: 1. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (((((real_t)(1.0000000000000000e+00)-((real_t)(2.0000000000000000e-02)*xd[2]))/(((real_t)(1.2000000000000000e+00)+(((real_t)(4.5454545454545456e-02)*xd[1])*xd[1]))+xd[1]))*(real_t)(4.7999999999999998e-01))*xd[1]);

/* Compute outputs: */
out[0] = (((real_t)(-1.4999999999999999e-01)*xd[0])+(xd[0]*a[0]));
out[1] = (((((real_t)(0.0000000000000000e+00)-xd[1])+u[0])*(real_t)(1.4999999999999999e-01))-(((real_t)(2.5000000000000000e+00)*xd[0])*a[0]));
out[2] = (((real_t)(-1.4999999999999999e-01)*xd[2])+(((real_t)(2.0000000000000001e-01)+((real_t)(2.2000000000000002e+00)*a[0]))*xd[0]));
out[3] = xd[0];
out[4] = u[0];
out[5] = ((real_t)(3.1249999999999997e-03)*xd[2]);
}

/* Fixed step size:0.24 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[6] = rk_eta[6];

for (run1 = 0; run1 < 10; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acado_rhs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 6 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[6] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[7] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[8] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[9] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[10] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.2000000000000000e-01*acadoWorkspace.rk_kkk[11] + rk_eta[5];
acado_rhs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 12 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)2.3999999999999999e-01*acadoWorkspace.rk_kkk[12] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)2.3999999999999999e-01*acadoWorkspace.rk_kkk[13] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)2.3999999999999999e-01*acadoWorkspace.rk_kkk[14] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)2.3999999999999999e-01*acadoWorkspace.rk_kkk[15] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)2.3999999999999999e-01*acadoWorkspace.rk_kkk[16] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)2.3999999999999999e-01*acadoWorkspace.rk_kkk[17] + rk_eta[5];
acado_rhs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 18 ]) );
rk_eta[0] += + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[0] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[6] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[12] + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[18];
rk_eta[1] += + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[1] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[7] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[13] + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[19];
rk_eta[2] += + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[2] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[8] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[14] + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[20];
rk_eta[3] += + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[3] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[9] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[15] + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[21];
rk_eta[4] += + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[4] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[10] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[16] + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[22];
rk_eta[5] += + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[5] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[11] + (real_t)7.9999999999999988e-02*acadoWorkspace.rk_kkk[17] + (real_t)3.9999999999999994e-02*acadoWorkspace.rk_kkk[23];
acadoWorkspace.rk_ttt += 1.0000000000000001e-01;
}
error = 0;
return error;
}

