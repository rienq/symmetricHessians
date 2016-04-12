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


#ifndef ACADO_COMMON_H
#define ACADO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_QPOASES  0
#define ACADO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC    4

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_QP_SOLVER ACADO_QPOASES

#include "acado_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_HARDCODED_CONSTRAINT_VALUES 0
/** Indicator for fixed initial state. */
#define ACADO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_N 20
/** Number of online data values. */
#define ACADO_NOD 0
/** Number of control variables. */
#define ACADO_NU 1
/** Number of differential variables. */
#define ACADO_NX 6
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_NY 0
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_NYN 0
/** Total number of QP optimization variables. */
#define ACADO_QP_NV 20
/** Number of integration steps per shooting interval. */
#define ACADO_RK_NIS 5
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RK_NSTAGES 2
/** Providing interface for arrival cost. */
#define ACADO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_WEIGHTING_MATRICES_TYPE 2


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct ACADOvariables_
{
int dummy;
/** Matrix of size: 21 x 6 (row major format)
 * 
 *  Matrix containing 21 differential variable vectors.
 */
real_t x[ 126 ];

/** Column vector of size: 20
 * 
 *  Matrix containing 20 control variable vectors.
 */
real_t u[ 20 ];

/** Column vector of size: 6
 * 
 *  Current state feedback vector.
 */
real_t x0[ 6 ];

/** Column vector of size: 20
 * 
 *  Lower bounds values.
 */
real_t lbValues[ 20 ];

/** Column vector of size: 20
 * 
 *  Upper bounds values.
 */
real_t ubValues[ 20 ];

/** Column vector of size: 6
 * 
 *  Lower bounds values for affine constraints.
 */
real_t lbAValues[ 6 ];

/** Column vector of size: 6
 * 
 *  Upper bounds values for affine constraints.
 */
real_t ubAValues[ 6 ];

/** Matrix of size: 20 x 6 (row major format) */
real_t mu[ 120 ];


} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
real_t rk_dim12_swap;

/** Column vector of size: 12 */
real_t rk_dim12_bPerm[ 12 ];

/** Column vector of size: 12 */
real_t rk_dim12_bPerm_trans[ 12 ];

/** Column vector of size: 200 */
real_t rhs_aux[ 200 ];

real_t rk_ttt;

/** Row vector of size: 7 */
real_t rk_xxx[ 7 ];

/** Matrix of size: 30 x 2 (row major format) */
real_t rk_Ktraj[ 60 ];

/** Matrix of size: 12 x 12 (row major format) */
real_t rk_A[ 144 ];

/** Column vector of size: 12 */
real_t rk_b[ 12 ];

/** Row vector of size: 12 */
int rk_dim12_perm[ 12 ];

/** Column vector of size: 6 */
real_t rk_rhsTemp[ 6 ];

/** Matrix of size: 2 x 42 (row major format) */
real_t rk_diffsTemp2[ 84 ];

/** Matrix of size: 210 x 2 (row major format) */
real_t rk_diffKtraj[ 420 ];

/** Matrix of size: 6 x 7 (row major format) */
real_t rk_diffsPrev2[ 42 ];

/** Matrix of size: 6 x 7 (row major format) */
real_t rk_diffsNew2[ 42 ];

/** Matrix of size: 60 x 12 (row major format) */
real_t rk_A_traj[ 720 ];

/** Matrix of size: 5 x 12 (row major format) */
int rk_aux_traj[ 60 ];

/** Matrix of size: 30 x 7 (row major format) */
real_t rk_S_traj[ 210 ];

/** Row vector of size: 60 */
real_t rk_stageV_traj[ 60 ];

/** Row vector of size: 12 */
real_t rk_b_trans[ 12 ];

/** Row vector of size: 55 */
real_t rk_seed[ 55 ];

/** Row vector of size: 34 */
real_t rk_adjoint[ 34 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t rk_hess1[ 49 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t rk_hess2[ 49 ];

/** Row vector of size: 83 */
real_t state[ 83 ];

/** Column vector of size: 120 */
real_t d[ 120 ];

/** Matrix of size: 120 x 6 (row major format) */
real_t evGx[ 720 ];

/** Column vector of size: 120 */
real_t evGu[ 120 ];

/** Matrix of size: 140 x 7 (row major format) */
real_t EH[ 980 ];

/** Matrix of size: 6 x 6 (row major format) */
real_t EH_N[ 36 ];

/** Column vector of size: 27 */
real_t objAuxVar[ 27 ];

/** Row vector of size: 7 */
real_t objValueIn[ 7 ];

/** Row vector of size: 43 */
real_t objValueOut[ 43 ];

/** Matrix of size: 120 x 6 (row major format) */
real_t Q1[ 720 ];

/** Column vector of size: 20 */
real_t R1[ 20 ];

/** Column vector of size: 120 */
real_t S1[ 120 ];

/** Matrix of size: 6 x 6 (row major format) */
real_t QN1[ 36 ];

/** Column vector of size: 126 */
real_t sbar[ 126 ];

/** Column vector of size: 6 */
real_t Dx0[ 6 ];

/** Column vector of size: 6 */
real_t W1[ 6 ];

/** Column vector of size: 6 */
real_t W2[ 6 ];

/** Column vector of size: 1260 */
real_t E[ 1260 ];

/** Column vector of size: 126 */
real_t QDy[ 126 ];

/** Column vector of size: 6 */
real_t w1[ 6 ];

/** Column vector of size: 6 */
real_t w2[ 6 ];

/** Matrix of size: 20 x 20 (row major format) */
real_t H[ 400 ];

/** Matrix of size: 6 x 20 (row major format) */
real_t A[ 120 ];

/** Column vector of size: 20 */
real_t g[ 20 ];

/** Column vector of size: 20 */
real_t lb[ 20 ];

/** Column vector of size: 20 */
real_t ub[ 20 ];

/** Column vector of size: 6 */
real_t lbA[ 6 ];

/** Column vector of size: 6 */
real_t ubA[ 6 ];

/** Column vector of size: 20 */
real_t x[ 20 ];

/** Column vector of size: 26 */
real_t y[ 26 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array of size 7 to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_acado_rhs(const real_t* in, real_t* out);

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_acado_diffs(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int acado_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int acado_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int acado_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void acado_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 21 with xEnd. 2. Initialize node 21 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t acado_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t acado_getObjective(  );

/** EVD-based regularization of a Hessian block.
 *
 */
void acado_regularize( real_t* const hessian_block );


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_COMMON_H */
