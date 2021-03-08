%% RFRFR Robot - Inverse Geometrico-Static Problem
% strong form diff. eqn solved with shooting method.
clear
% close all
clc
%% Parameters

E = 200*10^9; %young modulus
r = 0.001; %radius of circular cross sect.
I = .25*r^4*pi; %inertia moment
params.EI = E*I;
% fixed frame placed in [0;0]
params.pos10 = [-0.2;0]; % position of motor 1 (w.r.t. fixed frame)
params.pos20 = [+0.2;0]; % position of motor 2 (w.r.t. fixed frame)
params.L = 1; % beam's lenght
loads.fEnd = [0;0]; % end-eff. force
loads.distribforce = [0;0]; % distributed force
loads.distribmom = 0; % distributed torque
N = 100; % size of output data

%% Input EE position
pos_end = [0.3;0.7]; % end effector position

%% Solution
val = InverseShootingRFRFR(params,loads,pos_end,N);

%% Plot
PlotRFRFR(val(1:2,:),val(7:8,:),params.L)
%% Check
% sol = CheckDiffeq(val(1:6,:),1,N,E*I);
% s = linspace(0,1,N);
% figure()
% plot(s(2:end),sol(1,:))
% hold on
% plot(s(2:end),sol(2,:))
% plot(s(2:end),sol(3,:))
% plot(s(2:end),sol(4,:))
% plot(s(2:end),sol(5,:))
% plot(s(2:end),sol(6,:))