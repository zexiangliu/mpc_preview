% The MPC simulator for integrator dynamics
% Author: Zexiang Liu
clc; clear all; close all;
%% Parameters
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
R = 1e-12;
ref = zeros(n,H);
options = mpcqpsolverOptions;
% options = optimoptions('quadprog','Display','off');
% visual profile
visual_flag = 0;
td = 50;               % time steps between two displays
%% Main Simulation Loop
x_traj = x0;
r_traj = [0;0];
x = x0;
u_sig = [];
t = 0;
for i = 1:N-1
    t = t + tstep;
    % MPC control
    ref = ref_gen(t,n,H,tstep);
    [u_seq] = mpc_cont(A,B,x,H,ref,Q,R,options); 
    
    % state update
    d = [0.1,0.1];%(2*rand(2,1)-1)*0.5; % disturbance
    u = u_seq(:,1);
    x = A*x + B*u + d;
    
    % record
    x_traj = [x_traj,x];
    u_sig = [u_sig, u];
    r_traj = [r_traj,ref(:,1)];
    
    if visual_flag == 1
        if mod(i,td) == 0
            figure(1); hold on;
            plot([i-td:i]*tstep,x_traj(1,end-td:end),'b'); 
            plot([i-td:i]*tstep,r_traj(1,end-td:end),'r');
            figure(2); hold on;
            plot([i-td:i]*tstep,x_traj(2,end-td:end),'b'); 
            plot([i-td:i]*tstep,r_traj(2,end-td:end),'r');
            drawnow;
        end
    end
end

%% Evaluation
if visual_flag == 2
    figure(1); hold off;
    plot([0:i]*tstep,x_traj(1,:),'b'); hold on;
    plot([0:i]*tstep,r_traj(1,:),'r');
    figure(2); hold off;
    plot([0:i]*tstep,x_traj(2,:),'b'); hold on;
    plot([0:i]*tstep,r_traj(2,:),'r');
end
% compute total cost
cost = 0;
for i = 20:N-1
    cost = cost + x_traj(:,i)'*Q*x_traj(:,i+1) + u_sig(:,i)'*R*u_sig(:,i);
end
disp("The final cost is:"+num2str(cost));