function sol = CheckDiffeq(val,L,N,EI)
sd = L/N;
pd = (val(1:2,2:end)-val(1:2,1:(end-1)))/sd;
thd = (val(3,2:end)-val(3,1:(end-1)))/sd;
nd = (val(4:5,2:end)-val(4:5,1:(end-1)))/sd;
md = (val(6,2:end)-val(6,1:(end-1)))/sd;
eq1(1:2,:) = pd-[cos(val(3,2:end));sin(val(3,2:end))];
eq2 = thd - val(6,2:end)/EI;
eq3 = nd ;
eq4 = md + cos(val(3,2:end)).*val(5,2:end)-sin(val(3,2:end)).*val(4,2:end);

sol = [eq1;eq2;eq3;eq4];

end