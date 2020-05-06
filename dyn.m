function [A,B] = dyn(n)
% return (A,B) matrices of discrete-time dynamics of mutiple integrator
% input: n --- dimension

A = [zeros(n,1),[eye(n-1);zeros(1,n-1)]];
B = [zeros(n-1,1);1];
end