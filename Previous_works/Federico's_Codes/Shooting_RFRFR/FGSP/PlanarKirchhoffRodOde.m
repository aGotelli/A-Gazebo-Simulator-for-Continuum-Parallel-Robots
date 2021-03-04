%% Planar KirchhoffRodOde
%Differential equations for planar rod. Assume initially-striaght beam and
%neglect shear/extensibility. Non-minimal config. variables
function yd = PlanarKirchhoffRodOde(~,y,loads,EI)
% y = configuration variables vector = [px py theta nx ny mz]
% loads = structure with distributed load
% EI = material params
f = loads.distribforce;
l = loads.distribmom;
yd = zeros(6,1);
yd(1) = cos(y(3));
yd(2) = sin(y(3));
yd(3) = y(6)/(EI);
yd(4) = -f(1);
yd(5) = -f(2);
yd(6) = -yd(1)*y(5) + yd(2)*y(4)-l;
end