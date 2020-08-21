clear
clc

lambda=1*ones(12,1);

[lambda_solved,error] = fminsearch(@check_terminal,lambda);