
#include <acado_toolkit.hpp>

USING_NAMESPACE_ACADO

int main( ){
	
	// INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     X,S,P,X1, Sf1, P1;
    Control               Sf   ;
    IntermediateState     mu   ;
    DifferentialEquation  f    ;
    
    const double D       = 0.15;
    const double Ki      = 22.0;
    const double Km      = 1.2 ; 
    const double Pm      = 50.0;
    const double Yxs     = 0.4 ;
    const double alpha   = 2.2 ;
    const double beta    = 0.2 ;
    const double mum     = 0.48;
    const double Sfbar   = 32.7;
    const double Sfmin   = 28.7;
    const double Sfmax   = 40.0;
    const double Xbarmax = 5.8 ;

    const double t_start =  0.0;
    const double t_end   = 48.0;
    const uint N 		  = 20;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    mu = mum*(1.-P/Pm)*S/(Km+S+pow(S,2)/Ki);
    
    f << dot(X) == -D*X+mu*X;
    f << dot(S) == D*(Sf-S)-(mu/Yxs)*X;
    f << dot(P) == -D*P+(alpha*mu+beta)*X;
    f << dot(X1) == X;
    f << dot(Sf1) == Sf;
    f << dot(P1) == D*P/48.0;
    

	// DEFINE THE WEIGHTING MATRICES:
	// ----------------------------------------------------------
	Expression states;
	states << X;
	states << S;
	states << P;
	states << X1;
	states << Sf1;
	
	Function rf, rfN;
	rf << states;
	rf << Sf;
	
	rfN << states;
	
	BMatrix W(rf.getDim(), rf.getDim()); W.setAll( true );
	BMatrix WN(rfN.getDim(), rfN.getDim()); WN.setAll( true );
	// ----------------------------------------------------------


	// SET UP THE MPC - OPTIMAL CONTROL PROBLEM:
	// ----------------------------------------------------------
	OCP ocp( t_start, t_end, N );

	//ocp.minimizeLagrangeTerm( D*(0.05*Sf-P) );  // weight this with the physical cost!!!
    //ocp.minimizeLagrangeTerm( -D*P/48.0 );  // weight this with the physical cost!!!
    ocp.minimizeMayerTerm( -P1 );  // weight this with the physical cost!!!
	
    ocp.subjectTo( Sfmin <= Sf <= Sfmax );
	
    /* ocp.subjectTo( AT_START, X1 == 0 );
    ocp.subjectTo( AT_START, Sf1 == 0 );
    ocp.subjectTo( AT_START, P1 == 0 ); */
    
	ocp.subjectTo( AT_END, 0 <= X <= 0 );
	ocp.subjectTo( AT_END, 0 <= S <= 0 );
	ocp.subjectTo( AT_END, 0 <= P <= 0 );
    ocp.subjectTo( AT_END, 0 <= X1 <= 48*Xbarmax );
    ocp.subjectTo( AT_END, 0 <= Sf1 <= 48*Sfbar );
    ocp.subjectTo( AT_END, 0 <= P1 <= 1e8 );

	ocp.setModel( f );
	// ----------------------------------------------------------

//~ Logger::instance().setLogLevel( LVL_DEBUG );

	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,             INT_RK4   			);
	mpc.set( NUM_INTEGRATOR_STEPS,        100            		);
	mpc.set( QP_SOLVER,                   QP_QPOASES    		);
	mpc.set( HOTSTART_QP,                 NO             		);
	mpc.set( GENERATE_TEST_FILE,          NO            		);
	mpc.set( GENERATE_MAKE_FILE,          NO            		);
	mpc.set( GENERATE_MATLAB_INTERFACE,   YES            		);
	mpc.set( SPARSE_QP_SOLUTION, 		FULL_CONDENSING_N2	);
	//~ mpc.set( DYNAMIC_SENSITIVITY, 		FORWARD_OVER_BACKWARD	);
	mpc.set( DYNAMIC_SENSITIVITY, 		SYMMETRIC				);
	mpc.set( LEVENBERG_MARQUARDT, 		1e-10					);
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

	mpc.exportCode( "mpc_export" );
	mpc.printDimensionsQP( );
	// ----------------------------------------------------------


	// DEFINE A SIM EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	SIMexport sim( 1, t_end/N );
	
	sim.setModel( f );
	
	sim.set( INTEGRATOR_TYPE, 				INT_RK4 	);
	sim.set( NUM_INTEGRATOR_STEPS, 			10 			);
	sim.set( DYNAMIC_SENSITIVITY, 		NO_SENSITIVITY	);
	sim.set( GENERATE_MATLAB_INTERFACE,   	YES         );
	sim.exportCode( "sim_export" );
	// ----------------------------------------------------------

    return 0;
}



