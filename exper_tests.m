%% n = 2
clc;clear all;
% Parameters
tstep = 0.1;            % time step
Tspan = 20;            % time span 
N = Tspan/tstep+1;      % step number
% system profile
n = 2;                  % system dimension
x0 = [0;0];             % initial condition
[A,B] = dyn(n);         % get dynamics
% control profile
H = 100;                 % preview horizon
Q = eye(2);
R = 1;
% visual profile
visual_flag = 0;
td = 50;               % time steps between two displays

K = 5;
cost = zeros(K,1);
for i = 1:K
    H = i*10;
    cost(i) = simulator_run(tstep,Tspan,n,x0,A,B,H,Q,R,visual_flag,td);
end

cost

%% n = 3
clc;clear all;
% Parameters
tstep = 0.1;            % time step
Tspan = 20;            % time span 
N = Tspan/tstep+1;      % step number
% system profile
n = 5;                  % system dimension
x0 = 10*ones(n,1);             % initial condition
[A,B] = dyn(n);         % get dynamics
% control profile
Q = eye(n);
R = 1;
% visual profile
visual_flag = 0;
td = 50;               % time steps between two displays

cont = @(A,B,x,H,ref, Q,R,options) mpc_cont(A,B,x,H,ref,Q,R,options);
ref_sig = @(t,n,H,tstep) ref_gen(t,n,H,tstep);
d_sig = @(t,n,tstep) d_gen(t,n,tstep);
K = 5;
cost = zeros(K,1);
u_list = cell(K,1);
traj_list = cell(K,1);
for i = 1:K
    H = (i-1)*10+1;
    [cost(i),traj_list{i},u_list{i}] = simulator_run(tstep,Tspan,n,x0,A,B,H,Q,R,visual_flag,td,cont, ref_sig, d_sig);
end

cost

%% LQR

K = dlqr(A,B,Q,R,zeros(n,1));
cont = @(A,B,x,H,ref, Q,R,options) K*x;
[cost_lqr,traj_lqr,u_lqr] = simulator_run(tstep,Tspan,n,x0,A,B,H,Q,R,visual_flag,td,cont, ref_sig, d_sig);


%% n = 5, random Q, constant d

% clc;clear all;
% Parameters
tstep = 0.1;            % time step
Tspan = 20;            % time span 
N = Tspan/tstep+1;      % step number
% system profile
n = 5                   ;                  % system dimension
x0 = 10*ones(n,1);             % initial condition
[A,B] = dyn(n);         % get dynamics
% control profile
Q = rand(n);
Q = Q'*Q + eye(n);

R = 1;
% visual profile
visual_flag = 0;
td = 50;               % time steps between two displays
%%
d_siz = 0.1;
cont = @(A,B,x,H,ref, Q,R,options) mpc_cont(A,B,x,H,ref,Q,R,options);

ref_sig = @(t,n,H,tstep) ref_gen(t,n,H,tstep);
d_sig = @(t,n,tstep) d_siz*ones(n,1);
K = 20;
cost = zeros(K,1);
u_list = cell(K,1);
traj_list = cell(K,1);
horizon_list = zeros(K,1);
for i = 1:K
    H = (i-1)*1+1;
    horizon_list(i) = H;
    
    K_list = lqr_finite(A,B,H,Q,R);
    cont = @(A,B,x,H,ref,Q,R,options) K_list{1}*x;
    [cost(i),traj_list{i},u_list{i}] = simulator_run(tstep,Tspan,n,x0,A,B,H,Q,R,visual_flag,td,cont, ref_sig, d_sig);
end

plot(horizon_list,cost,'*-');hold on;
title("dimension="+num2str(n)+", disturbance="+num2str(d_siz));
xlabel('MPC horizon');
ylabel('Execution cost');

%% n = 5, random Q, randomized d

clc;clear all;
% Parameters
tstep = 0.1;            % time step
Tspan = 20;            % time span 
N = Tspan/tstep+1;      % step number
% system profile
n = 5                   ;                  % system dimension
x0 = 10*ones(n,1);             % initial condition
[A,B] = dyn(n);         % get dynamics
% control profile
Q = rand(n);
Q = Q'*Q + eye(n);

R = 1;
% visual profile
visual_flag = 0;
td = 50;               % time steps between two displays

cont = @(A,B,x,H,ref, Q,R,options) mpc_cont(A,B,x,H,ref,Q,R,options);
ref_sig = @(t,n,H,tstep) ref_gen(t,n,H,tstep);
K = 20;

L = 10; % # rounds
COST = cell(L,1);
U_list = cell(L,1);
Traj_list = cell(L,1);
D_list = cell(L,1);
for j = 1:L
    
    d_siz = 0.005;
    d_profile = d_siz*(2*rand(n,N)-1);
    d_sig = @(t,n,tstep) d_profile(:,round(t/tstep)+1);
    

    cost = zeros(K,1);
    u_list = cell(K,1);
    traj_list = cell(K,1);
    horizon_list = zeros(K,1);
    for i = 1:K
        H = (i-1)*1+1;
        horizon_list(i) = H;
        [cost(i),traj_list{i},u_list{i}] = simulator_run(tstep,Tspan,n,x0,A,B,H,Q,R,visual_flag,td,cont, ref_sig, d_sig);
    end

    COST{j} = cost;
    U_list{j} = u_list;
    Traj_list{j} = traj_list;
    
    figure(1); hold on;
    plot(horizon_list,cost,'.-','linewidth',1.5)
    title("dimension="+num2str(n)+", random disturbance with bound "+num2str(d_siz));
    xlabel('MPC horizon');
    ylabel('Execution cost');
    set(gca, 'YScale', 'log');
    drawnow;
end

%%

for j=1:9
    figure(1); hold on;
    plot(horizon_list,COST{j},'.-','linewidth',1.5)
    title("dimension="+num2str(n)+", random disturbance with bound "+num2str(d_siz));
    xlabel('MPC horizon');
    ylabel('Execution cost');
end