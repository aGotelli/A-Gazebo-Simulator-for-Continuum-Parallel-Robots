%% ForwardShootingRFRFR
% Solve Inverse-problem differential equations with shooting method
function val = InverseShootingRFRFR(params,loads,pos_end,N)
% params: structure with robot-parameters
% loads: structure with loads parameters
% pos_end = end-effector position (w.r.t fixed frame in [0;0])
% N = size of the output vector
global sol1;
global sol2;
EI = params.EI;
p10 = params.pos10;
p20 = params.pos20;
L = params.L;
fEnd = loads.fEnd;
y0 = [pi/2;zeros(3,1);pi/2;zeros(3,1)]; %guess initial angles as pi/2
fsolve(@(guess) ShootEqnRFRFRInv(guess),y0); % solve shooting equations
sInt = linspace(0,L,N);
val = [spline(sol1.x,sol1.y,sInt);spline(sol2.x,sol2.y,sInt)]; %uniform values

% Shooting Equations for Forward problem
function res = ShootEqnRFRFRInv(guess)
    th10 = guess(1,1);
    n10 = guess(2:3,1);
    m10 = guess(4,1);
    th20 = guess(5,1);
    n20 = guess(6:7,1);
    m20 = guess(8,1);
    res = zeros(8,1);
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
    res(7:8,1) = sol1.y(1:2,end) - pos_end; % check end effector position
end
end