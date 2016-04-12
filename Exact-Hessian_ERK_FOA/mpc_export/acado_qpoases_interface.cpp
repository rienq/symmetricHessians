/*
 *    This file was auto-generated by ACADO Code Generation Tool.
 *    
 *    ACADO Code Generation tool is a sub-package of ACADO toolkit --
 *    A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *    
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *    
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *    
 */


extern "C"
{
#include "acado_common.h"
}

#include "INCLUDE/QProblem.hpp"

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
#include "INCLUDE/EXTRAS/SolutionAnalysis.hpp"
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

static int acado_nWSR;



#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis acado_sa;
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

int acado_solve( void )
{
	acado_nWSR = QPOASES_NWSRMAX;

	QProblem qp(20, 6);
	
	returnValue retVal = qp.init(acadoWorkspace.H, acadoWorkspace.g, acadoWorkspace.A, acadoWorkspace.lb, acadoWorkspace.ub, acadoWorkspace.lbA, acadoWorkspace.ubA, acado_nWSR);

    qp.getPrimalSolution( acadoWorkspace.x );
    qp.getDualSolution( acadoWorkspace.y );
	
#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;
		
	retVal = acado_sa.getHessianInverse( &qp, );

#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int acado_getNWSR( void )
{
	return acado_nWSR;
}

const char* acado_getErrorString( int error )
{
	return MessageHandling::getErrorString( error );
}
