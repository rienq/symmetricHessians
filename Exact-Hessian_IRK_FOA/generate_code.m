
% Introduce variables
DifferentialState X S P X1 Sf1 P1;
Control Sf;

% Define constants
D       = 0.15;
Ki      = 22.0;
Km      = 1.2 ;
Pm      = 50.0;
Yxs     = 0.4 ;
alpha   = 2.2 ;
beta    = 0.2 ;
mum     = 0.48;
Sfbar   = 32.7;
Sfmin   = 28.7;
Sfmax   = 40.0;
Xbarmax = 5.8 ;

t_start =  0.0;
t_end   = 48.0;
N 		= 20;
Ts      = t_end/N;

%% Differential Equation
mu = is(mum*(1.-P/Pm)*S/(Km+S+(S*S)/Ki));

f = [   dot(X) == -D*X+mu*X; ...
        dot(S) == D*(Sf-S)-(mu/Yxs)*X; ...
        dot(P) == -D*P+(alpha*mu+beta)*X; ...
        dot(X1) == X; ...
        dot(Sf1) == Sf; ...
        dot(P1) == D*P/48.0  ];

%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 10;
sim = acado.SIMexport( Ts );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_RK4'       );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );
sim.set( 'DYNAMIC_SENSITIVITY',        'NO_SENSITIVITY' );

sim.exportCode( 'sim_export' );

%% MPCexport
acadoSet('problemname', 'mpc');

ocp = acado.OCP( 0.0, N*Ts, N );
ocp.minimizeMayerTerm( -P1 );

ocp.subjectTo( Sfmin <= Sf <= Sfmax );

ocp.subjectTo( 'AT_END', 0 <= X <= 0 );
ocp.subjectTo( 'AT_END', 0 <= S <= 0 );
ocp.subjectTo( 'AT_END', 0 <= P <= 0 );
ocp.subjectTo( 'AT_END', 0 <= X1 <= 48*Xbarmax );
ocp.subjectTo( 'AT_END', 0 <= Sf1 <= 48*Sfbar );
ocp.subjectTo( 'AT_END', 0 <= P1 <= 1e8 );

ocp.setModel( f );

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'EXACT_HESSIAN'          );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING'      );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2'     );
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'            );
mpc.set( 'DYNAMIC_SENSITIVITY', 		'FORWARD_OVER_BACKWARD'  );
mpc.set( 'NUM_INTEGRATOR_STEPS',        100                      );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'             );
mpc.set( 'HOTSTART_QP',                 'NO'                     );
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10                   );
mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES', 'NO'                   );

mpc.exportCode( 'mpc_export' );

