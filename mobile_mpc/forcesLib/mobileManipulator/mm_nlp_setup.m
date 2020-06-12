%% Clean up
clear; clc; close all; clearvars;
rng('shuffle');

solverName = 'mm_MPC';


disp("DEFINE THE PATH TO YOUR FORCES AND CASADI INSTALLATION");
pathForces = '/home/mspahn/develop/forces';
pathCasadi = '/home/mspahn/develop/casadi';

forcesPath = genpath(pathForces);
casadiPath = genpath(pathCasadi);
addpath(forcesPath);
addpath(casadiPath);
addpath('functionsOptimization');
addpath('dyn_model_panda');

% Turn off warnings for having a too recent Gcc compiler and some
% SoapService
warning('off', 'MATLAB:mex:GccVersion_link');
warning('off', 'MATLAB:webservices:WSDLDeprecationWarning');

%% Delete previous Solver
% Forces does not always code changes and might reuse the previous solution
try
FORCEScleanup('solverName','all');
catch
end

try
    rmdir('@FORCESproWS','s')
catch
end
try
    rmdir('solverName','s')
catch
end
% 
%% Some utility functions
deg2rad = @(deg) deg/180*pi; % convert degrees into radians
rad2deg = @(rad) rad/pi*180; % convert radians into degrees

%% Problem dimensions
model.N = 15;                                   % horizon length
model.nvar = 3 + 7 + 7 + 2 + 7;                     % number of variables [x, y, theta, q (size : 7), q_dot (size : 7), u1, u2, tau (size : 7)]
model.neq= 3 + 7 + 7;                               % dimension of transition function
nbObstacles = 1;
nbSpheres = 6;                                  % base + 5 for the arm
model.nh = nbObstacles * nbSpheres;             % number of inequality constraint functions
n_other_param = 3 + 3 + 7 + 7 + 4 * nbObstacles;    % [dt, r, L, x_des, y_des, theta_des, q_des (size : 7), q_vel_des (size : 7), obstacles(1).x, obstacles(1).y, obstacle(3), obsctacles(1).r, ...]

model.npar =  n_other_param;          % number of parameters

%% Limits for robot
q_lim_franka_up = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
q_lim_franka_low = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
q_lim_franka_vel = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
q_lim_franka_torque = [87, 87, 87, 87, 12, 12, 12];
q_lim_franka_acc = [15, 7.5, 10, 12.5, 15, 20, 20, 20];

% [x, y, theta, q u1, u2];
lower_bound = [-inf, -inf, -pi, q_lim_franka_low, -q_lim_franka_vel, -100, -100, -q_lim_franka_torque];
upper_bound = [inf, inf, pi, q_lim_franka_up, q_lim_franka_vel, 100, 100, q_lim_franka_torque];
model.lb = lower_bound;
model.ub = upper_bound;

%%
model.objective = @(z, p) costFunction(z, p);
model.ineq = @(z, p) obstacleAvoidance(z, p);
model.hu = inf(nbObstacles * nbSpheres, 1);
model.hl = zeros(nbObstacles * nbSpheres, 1);

%% Dynamics, i.e. equality constraints 
%model.objective = @(z, p) objective_scenario_try(z, p);
model.eq = @(z, p) transitionFunction(z, p);

model.E = [eye(17, 17), zeros(17, 9)];

%% Initial and final conditions
% Initial condition on vehicle states

model.xinitidx = 1:26; % use this to specify on which variables initial conditions are imposed
%model.xfinal = 0; % v final=0 (standstill), heading angle final=0?
%model.xfinalidx = 6; % use this to specify on which variables final conditions are imposed

%% Define solver options
codeoptions = getOptions(solverName);
codeoptions.maxit = 250;   % Maximum number of iterations
codeoptions.printlevel = 0 ; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.timing = 0;
codeoptions.overwrite = 1;
codeoptions.mu0 = 20;
codeoptions.cleanup = 1;
codeoptions.BuildSimulinkBlock = 0;
%codeoptions.nlp.lightCasadi = 1;

FORCES_NLP(model, codeoptions);
