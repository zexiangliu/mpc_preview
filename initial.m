clc;clear all;close all;
path(pathdef);
cd ..
% set up mosek
addpath(genpath('../mosek/8/toolbox/'))
javaaddpath ../mosek/8/tools/platform/linux64x86/bin/mosekmatlab.jar

% set up gurobi
cd '/home/zexiang/PROG/gurobi8.1.0_linux64/gurobi810/linux64/matlab'
gurobi_setup

cd '/home/zexiang/PROG/PCIS/pcis_projects'

cd tbxmanager/
startup
clc;

cd '/home/zexiang/PROG/PCIS/mpc_preview'
mpt_init

addpath(genpath('./'))