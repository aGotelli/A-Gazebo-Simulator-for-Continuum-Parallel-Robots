%% RFRFR Robot - Forward Geometrico-Static Problem
% strong form diff. eqn solved with shooting method.
clear
% close all
clc
%% Parameters
E = 200*10^9; %young modulus
r = 0.001; %radius of circular cross sect.
I = .25*r^4*pi; %inertia moment
params.EI = E*I;
params.pos10 = [-0.2;0]; % position of motor 1
params.pos20 = [0.2;0]; % position of motor 2
params.L = 1; % beam's lenght
loads.fEnd = [0;0]; % end-eff. force
loads.distribforce = [0;0]; % distributed force
loads.distribmom = 0; % distributed torque
N = 100; % size of output data

%% Input Angles
th10 = 120*pi/180; % actuation angle 1
th20 = 60*pi/180; % actuation angle 2

%% Solution
val = ForwardShootingRFRFR(params,loads,th10,th20,N);

%% Plot
PlotRFRFR(val(1:2,:),val(7:8,:),params.L)