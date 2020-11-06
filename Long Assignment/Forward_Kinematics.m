%% Forward Kinematics

a1=0.2; a2=0.25;
r=size(simout,1);

px=zeros(r,1);
py=zeros(r,1);
pz=zeros(r,1);

for i = 1:r
    theta1=simout(i,1);
    theta2=simout(i,2);
    theta3=simout(i,3);
    d4=simout(i,4); 
    
    px(i,1) = (a1*cos(theta1))+(a2*cos(theta1+theta2));
    py(i,1) = (a1*sin(theta1))+(a2*sin(theta1+theta2));
    pz(i,1) = -d4;
end

P=readmatrix("Waypoints.xlsx");

plot3(P(:,2),P(:,3),P(:,4));
hold on;
plot3(px,py,pz,'--');

grid on;
grid minor;
legend({'Actual Trajectory','Estimated Trajectory'},'Location','best');
xlabel("X(t)");
ylabel("Y(t)");
zlabel("Z(t)");

%%
syms theta1 theta2 theta3 a1 a2 d3 d4;
T01 = [ cos(theta1),-sin(theta1),0,a1*cos(theta1); sin(theta1),cos(theta1),0,a1*sin(theta1); 0,0,1,0; 0,0,0,1 ];
T12 = [ cos(theta2),sin(theta2),0,a2*cos(theta2); sin(theta2),-cos(theta2),0,a2*sin(theta2); 0,0,-1,0; 0,0,0,1 ];
T23 = [ cos(theta3),-sin(theta3),0,0; sin(theta3),cos(theta3),0,0; 0,0,1,d3; 0,0,0,1 ];
T32 = [ 1,0,0,0; 0,1,0,0; 0,0,1,d4; 0,0,0,1 ];
T04 = T01*T12*T23*T32;
