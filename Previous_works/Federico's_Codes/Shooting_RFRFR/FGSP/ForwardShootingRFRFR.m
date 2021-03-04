%% ForwardShootingRFRFR
% Solve Forward-problem differential equations with shooting method
function val = ForwardShootingRFRFR(params,loads,th10,th20,N)
% params: structure with robot-parameters
% loads: structure with loads parameters
% th10 = motor 1 angle (radiant)
% th20 = motor 2 angle (radiant)
% N = size of the output vector
global sol1;
global sol2;
EI = params.EI;
p10 = params.pos10;
p20 = params.pos20;
L = params.L;
fEnd = loads.fEnd;
y0 = zeros(6,1);
fsolve(@(guess) ShootEqnRFRFRForw(guess),y0); % solve shooting equations
sInt = linspace(0,L,N);
val = [spline(sol1.x,sol1.y,sInt);spline(sol2.x,sol2.y,sInt)]; %uniform values

% Shooting Equations for Forward problem
function res = ShootEqnRFRFRForw(guess)
    n10 = guess(1:2,1);
    m10 = guess(3,1);
    n20 = guess(4:5,1);
    m20 = guess(6,1);
    res = zeros(6,1);
    fun = @(s,y) PlanarKirchhoffRodOde(s,y,loads,EI);
    y01 = [p10;th10;n10;m10]; % quessed + imposed values
    y02 = [p20;th20;n20;m20]; % guessed + imposed values
    Lspan = [0,L];
    sol1 = ode45(fun,Lspan,y01); %integrate over L
    sol2 = ode45(fun,Lspan,y02);
    res(1:2,1) = sol1.y(1:2,end) - sol2.y(1:2,end); %check closure loop
    res(3:4,1) = sol1.y(4:5,end) + sol2.y(4:5,end) + fEnd; % check equilibrium
    res(5,1) = sol1.y(6,end); % check torque = 0 at end (revolute joint)
    res(6,1) = sol2.y(6,end); %check torque = 0 at end (revolute joint)
end
end