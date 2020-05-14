function params=load_params()
% 机体参数
params.m = 1;
params.g = 9.8;
params.k = 0.2;
params.kd = 0;
params.I = diag([0.04 0.04 0.008]);
params.L = 0.5;
params.b = 1e-4;
end
