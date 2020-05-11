 % n = 5 robust MPC, random A,B
clc;clear all;
% Parameters
tstep = 0.1;            % time step
Tspan = 20;            % time span 
N = Tspan/tstep+1;      % step number
% system profile
n = 5                   ;                  % system dimension
x0 = 10*ones(n,1);             % initial condition
[A,B] = dyn_rand(n);         % get dynamics
% control profile
Q = rand(n);
Q = Q'*Q + eye(n);

R = 1;
% visual profile
visual_flag = 0;
td = 50;               % time steps between two displays

d_siz = 0.05;
cont = @(A,B,x,H,ref, Q,R) robust_mpc(A,B,x,H,ref,Q,R,-d_siz,d_siz);
ref_sig = @(t,n,H,tstep) ref_gen(t,n,H,tstep);
K = 20;

L = 1; % # rounds
COST = cell(L,1);
U_list = cell(L,1);
Traj_list = cell(L,1);
D_list = cell(L,1);
for j = 1:L
    
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
end


cost2 = zeros(K,1);
u_list2 = cell(K,1);
traj_list2 = cell(K,1);
horizon_list2 = zeros(K,1);
for i = 1:K
    H = (i-1)*1+1;
    horizon_list2(i) = H;
    K_list = lqr_finite(A,B,H,Q,R);
    cont = @(A,B,x,H,ref,Q,R,options) K_list{1}*x;
    [cost2(i),traj_list2{i},u_list2{i}] = simulator_run(tstep,Tspan,n,x0,A,B,H,Q,R,visual_flag,td,cont, ref_sig, d_sig);
end

% figure(1); hold on;
% plot(horizon_list,cost,'+-','linewidth',1.5)
% plot(horizon_list2,cost2,'.-','linewidth',1.5)
% title("dimension="+num2str(n)+", random disturbance with bound "+num2str(d_siz));
% xlabel('MPC horizon');
% ylabel('Execution cost');
% set(gca, 'YScale', 'log');
% legend('MPC', 'robust MPC')
% 
% 
% figure(2); hold on;
% plot(horizon_list,abs(cost-cost2),'+-','linewidth',1.5)
% xlabel('MPC horizon');
