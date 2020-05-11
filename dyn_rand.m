function [A,B] = dyn_rand(n)
% randomly generate controllable (A,B) matrices 
% input: n --- dimension

A = [zeros(n,1),[eye(n-1);zeros(1,n-1)]];
A(end,:) = rand(1,n);
B = [zeros(n-1,1);1];
end