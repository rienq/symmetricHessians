clc;
clear all;
close all;

COMPILE = 0;

if COMPILE
    cd mpc_export
    make_rien_solver('../acado_MPCstep')
    cd ..
    cd sim_export
    make_acado_integrator('../acado_simulate')
    cd ..
end

Fontsize = 20;
set(0,'DefaultAxesFontSize',Fontsize)

%% PARAMETERS SIMULATION
N = 20;
Ts = 48/N;

Sfmin   = 28.7;
Sfmax   = 40.0;
% Sfbar   = 32.7;
% Xbarmax = 5.8 ;
Sfbar   = 33;
Xbarmax = 5.8 ;

% D = 0.15;

init_states = textread('init_states.txt', '');
init_controls = textread('init_controls.txt', '');

% X0 = [5.7 13 20 zeros(1,3)];
X0 = [init_states(1,2:4) zeros(1,3)];
Xref = [0 0 0 0 0 0];
input.x = repmat(X0,N+1,1);
% for i = 1:size(init_states,1)-1
%     input.x = [input.x; repmat(init_states(i,2:6),2,1)];
% end
% input.x = [input.x; repmat(init_states(end,2:6),1,1)];
Xref = repmat(Xref,N,1);

input.p = [];
input.mu = zeros(N,length(X0));

Uref = zeros(N,1);
input.u = zeros(N,1);
% for i = 1:size(init_controls,1)-1
%     input.u = [input.u; repmat(init_controls(i,2),2,1)];
% end

input.lbValues = Sfmin*ones(N,1);
input.ubValues = Sfmax*ones(N,1);

input.lbAValues = [X0(1:3).'; 0; 0; 0];
input.ubAValues = [X0(1:3).'; 48*Xbarmax; 48*Sfbar; 1e8];

% input.lbAValues = [0; 0; 0; 0; 0; 0];
% input.ubAValues = [1e2;1e2;1e2; 48*Xbarmax; 48*Sfbar; 1e8];

input.x0 = [X0(1:3) zeros(1,3)];

%% SOLVE FIRST OCP

totalCPU = 0;
totalMODEL = 0;
totalCONDENSE = 0;
totalREGULARIZE = 0;
totalQP = 0;
totalSWEEP1 = 0;
totalSWEEP2 = 0;
totalSWEEP3 = 0;
KKT = 1;
iter = 0;
outputs = {};
while KKT > 1e-14
    output = acado_MPCstep(input);
    
    input.x = output.x;
    input.u = output.u;
    input.mu = output.mu;
    
    iter = iter+1;
    disp(['iter ' num2str(iter) ': QP status = ' num2str(output.info.status) ', KKT val = ' num2str(output.info.kktValue) ', cpu Time = ' num2str(output.info.cpuTime)])
    
    KKT = output.info.kktValue;
    outputs{iter} = output;
    
    totalCPU = totalCPU + output.info.cpuTime;
    totalMODEL = totalMODEL + output.info.modelTime;
    totalCONDENSE = totalCONDENSE + output.info.condenseTime;
    totalREGULARIZE = totalREGULARIZE + output.info.regularizeTime;
    totalQP = totalQP + output.info.qpTime;
    
    totalSWEEP1 = totalSWEEP1 + output.info.sweep1Time;
    totalSWEEP2 = totalSWEEP2 + output.info.sweep2Time;
    totalSWEEP3 = totalSWEEP3 + output.info.sweep3Time;
end

sol = output; err = [];
for i = 1:length(outputs)-1
   err_x = max(max(abs(outputs{i}.x - sol.x)));
   err_u = max(max(abs(outputs{i}.u - sol.u)));
   err = [err max(err_x, err_u)];
end

figure;
semilogy(err, '--ko','linewidth',1.6,'MarkerSize',12);
xlabel('Iteration number')
ylabel('|| W - W^* ||_\infty')


disp('----------------------------------------------');
disp('----------------------------------------------');
disp(['AVERAGE RTI step CPU time      : ' num2str(round(1e6*totalCPU/iter)) ' μs']);
disp('----------------------------------------------');
disp(['time spent in forward sweep 1  : ' num2str(round(1e6*totalSWEEP1/iter)) ' μs (' num2str(round(100*totalSWEEP1/totalCPU)) '%)']);
disp(['time spent in backward sweep 2 : ' num2str(round(1e6*totalSWEEP2/iter)) ' μs (' num2str(round(100*totalSWEEP2/totalCPU)) '%)']);
disp(['time spent in forward sweep 3  : ' num2str(round(1e6*totalSWEEP3/iter)) ' μs (' num2str(round(100*totalSWEEP3/totalCPU)) '%)']);
disp('----------------------------------------------');
disp(['time spent in the model        : ' num2str(round(1e6*totalMODEL/iter)) ' μs (' num2str(round(100*totalMODEL/totalCPU)) '%)']);
disp(['time spent in condensing       : ' num2str(round(1e6*totalCONDENSE/iter)) ' μs (' num2str(round(100*totalCONDENSE/totalCPU)) '%)']);
disp(['time spent in regularization   : ' num2str(round(1e6*totalREGULARIZE/iter)) ' μs (' num2str(round(100*totalREGULARIZE/totalCPU)) '%)']);
disp(['time spent in QPoases          : ' num2str(round(1e6*totalQP/iter)) ' μs (' num2str(round(100*totalQP/totalCPU)) '%)']);
disp('----------------------------------------------');
disp('----------------------------------------------');

TIME = [0:Ts:N*Ts];
X = output.x.';
U = output.u.';

D = 0.15;

figure; clf;
    label = {'X_b (g/L)','X_s (g/L)','X_p (g/L)'};
    idx = [1,2,3];
    plotI = [1 3 2];
    for k = 1:length(plotI)
        subplot(2,2,plotI(k))
        hold on
            plot(TIME,X(idx(k),:),'-ko','linewidth',1.6)
%             plot(TIME,X2(idx(k),:),'k:','linewidth',1.6)
            if k == 1
                plot([TIME(1) TIME(end)],[Xbarmax Xbarmax],'k--');
            elseif k == 3
                plot([TIME(1) TIME(end)],[input.x(end,end) input.x(end,end)]./D,'k--');
            end
            ylabel(label{k})
            xlabel('time (h)')
    end
      
subplot(2,2,4)
        stairs(TIME(1:end-1),U(1,:),'-ko','linewidth',1.6)
    hold on
%         stairs(TIME(1:end-1),U2(1,:),'k:','linewidth',1.6)
        plot([TIME(1) TIME(end-1)], [Sfmin Sfmin], 'k--');
        plot([TIME(1) TIME(end-1)], [Sfmax Sfmax], 'k--');
        plot([TIME(1) TIME(end-1)],[Sfbar Sfbar],'k--');
            ylabel('U_f (g/L)')
    xlabel('time (h)')
    ylim([27 42]);
    xlim([0 48]);


%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

Tf = 480;
Te = 240; EVENT = 1;

time = 0;
KKT_MPC = []; INFO_MPC = []; OBJ_MPC = [];
controls_MPC = [];
state_sim = X0;

totalCPU = 0;
totalMODEL = 0;
totalCONDENSE = 0;
totalREGULARIZE = 0;
totalQP = 0;
totalSWEEP1 = 0;
totalSWEEP2 = 0;
totalSWEEP3 = 0;
iter = 0;
% pause
while time(end) < Tf
% while iter < 10
    tic
    % Solve NMPC OCP
    input.x0 = [state_sim(end,1:3) zeros(1,3)];
%     input.od = D;
    input.lbAValues = [state_sim(end,1:3).'; 0; 0; 0];
    input.ubAValues = [state_sim(end,1:3).'; 48*Xbarmax; 48*Sfbar; 1e8];
    for i = 1:1
        output = acado_MPCstep(input);
        
        input.x = output.x;
        input.u = output.u;
        input.mu = output.mu;
        
%         disp(['RTI step ' num2str(i) ' KKT value: ' num2str(output.info.kktValue)]);
    end
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    
    % SAVE THE OBJECTIVE VALUE AT THE END OF THE STATES
    OBJ_MPC = [OBJ_MPC; input.x(end,end)];   % ---> average productivity for this OCP
    
    % SHIFTING !!
    input.x = [input.x(2:end,:); input.x(2,:)];
    input.u = [input.u(2:end,:); input.u(1,:)];
    input.mu = [input.mu(2:end,:); input.mu(2,:)];
    
    % Simulate system
    input_sim.x = [state_sim(end,1:3).'; zeros(3,1)];
    input_sim.u = output.u(1,:);
    states = acado_simulate(input_sim);
    
    if( time(end) >= Te && EVENT )
        % event on X:
        states.value(1:3) = [6.1 14 21.1].';
        
%         D = 0.152;
        
        EVENT = 0;
    end
    state_sim = [state_sim; states.value'];
    
    nextTime = iter*Ts; 
    disp(['current time      : ' num2str(nextTime)])
    disp(['RTI step KKT value: ' num2str(output.info.kktValue)]);
    disp(['RTI step CPU time : ' num2str(round(1e6*output.info.cpuTime)) ' μs']);
    disp('----------------------------------------------');
    time = [time nextTime];
    iter = iter+1;
    
    totalCPU = totalCPU + output.info.cpuTime;
    totalMODEL = totalMODEL + output.info.modelTime;
    totalCONDENSE = totalCONDENSE + output.info.condenseTime;
    totalREGULARIZE = totalREGULARIZE + output.info.regularizeTime;
    totalQP = totalQP + output.info.qpTime;
    
    totalSWEEP1 = totalSWEEP1 + output.info.sweep1Time;
    totalSWEEP2 = totalSWEEP2 + output.info.sweep2Time;
    totalSWEEP3 = totalSWEEP3 + output.info.sweep3Time;
    
%     pause(abs(Ts-toc));
end

save ../sim_data.mat state_sim controls_MPC

disp('----------------------------------------------');
disp('----------------------------------------------');
disp(['AVERAGE RTI step CPU time      : ' num2str(round(1e6*totalCPU/iter)) ' μs']);
disp('----------------------------------------------');
disp(['time spent in forward sweep 1  : ' num2str(round(1e6*totalSWEEP1/iter)) ' μs (' num2str(round(100*totalSWEEP1/totalCPU)) '%)']);
disp(['time spent in backward sweep 2 : ' num2str(round(1e6*totalSWEEP2/iter)) ' μs (' num2str(round(100*totalSWEEP2/totalCPU)) '%)']);
disp(['time spent in forward sweep 3  : ' num2str(round(1e6*totalSWEEP3/iter)) ' μs (' num2str(round(100*totalSWEEP3/totalCPU)) '%)']);
disp('----------------------------------------------');
disp(['time spent in the model        : ' num2str(round(1e6*totalMODEL/iter)) ' μs (' num2str(round(100*totalMODEL/totalCPU)) '%)']);
disp(['time spent in condensing       : ' num2str(round(1e6*totalCONDENSE/iter)) ' μs (' num2str(round(100*totalCONDENSE/totalCPU)) '%)']);
disp(['time spent in regularization   : ' num2str(round(1e6*totalREGULARIZE/iter)) ' μs (' num2str(round(100*totalREGULARIZE/totalCPU)) '%)']);
disp(['time spent in QPoases          : ' num2str(round(1e6*totalQP/iter)) ' μs (' num2str(round(100*totalQP/totalCPU)) '%)']);
disp('----------------------------------------------');
disp('----------------------------------------------');


%% Plot

X = state_sim.';
U = controls_MPC.';

TIME = (1:Tf/Ts+1)*Ts;
TIME = time;

load ../sol_data.mat state_sim controls_MPC

X2 = state_sim.';
U2 = controls_MPC.';

figure; clf;
    label = {'X (g/L)','S (g/L)','P (g/L)'};
    idx = [1,2,3];
    plotI = [1 3 2];
    for k = 1:length(plotI)
        subplot(2,2,plotI(k))
        hold on
            plot(TIME,X(idx(k),:),'k','linewidth',1.6)
            plot(TIME,X2(idx(k),:),'k:','linewidth',1.6)
            if k == 1
                plot([TIME(1) TIME(end)],[Xbarmax Xbarmax],'k--');
            elseif k == 3
                plot(TIME(1:end-1),OBJ_MPC./D,'k--');
            end
            ylabel(label{k})
            xlabel('time (h)')
    end
      
subplot(2,2,4)
        stairs(TIME(1:end-1),U(1,:),'k','linewidth',1.6)
    hold on
        stairs(TIME(1:end-1),U2(1,:),'k:','linewidth',1.6)
        plot([TIME(1) TIME(end-1)], [Sfmin Sfmin], 'k--');
        plot([TIME(1) TIME(end-1)], [Sfmax Sfmax], 'k--');
        plot([TIME(1) TIME(end-1)],[Sfbar Sfbar],'k--');
            ylabel('S_f (g/L)')
    xlabel('time (h)')
    ylim([27 42]);
    
    