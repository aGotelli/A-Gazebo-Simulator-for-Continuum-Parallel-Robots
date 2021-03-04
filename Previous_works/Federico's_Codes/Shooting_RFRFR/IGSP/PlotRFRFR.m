function PlotRFRFR(pos1,pos2,L)
% figure()
plot(pos1(1,:),pos1(2,:),'k-','LineWidth',1.5)
hold on
plot(pos2(1,:),pos2(2,:),'k-','LineWidth',1.5)
plot(pos1(1,1),pos1(2,1),'ro','LineWidth',2)
plot(pos2(1,1),pos2(2,1),'ro','LineWidth',2)
plot(pos2(1,end),pos2(2,end),'ko','LineWidth',2)
quiver(0,0,L/5,0,'b','LineWidth',2)
quiver(0,0,0,L/5,'b','Linewidth',2)
axis equal
grid on
end