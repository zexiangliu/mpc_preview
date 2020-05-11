# MPC Preview

To run the code, one need to add the current folder and YALMIP in the search path.

Some test codes are available in file `exper_tests.m`.

## Usage
The main simulation program is  `simulator_run`.

There are three controllers available: MPC, finitie-horizon LQR and robust MPC. In theory, given the same Q, R and H, MPC and LQR should give the same results. One can define a function handler for one of the three controllers as follows. The function handler will be a input of `simulation_run`.
``` 
% MPC
cont = @(A,B,x,H,ref, Q,R) mpc_cont(A,B,x,H,ref,Q,R,options);

% Finite-horizon LQR
K_list = lqr_finite(A,B,H,Q,R);
cont = @(A,B,x,H,ref,Q,R,options) K_list{1}*x;

% Robust MPC (disturbance range=[d_min, d_max])
cont = @(A,B,x,H,ref, Q,R) robust_mpc(A,B,x,H,ref,Q,R,d_min,d_max);
```
