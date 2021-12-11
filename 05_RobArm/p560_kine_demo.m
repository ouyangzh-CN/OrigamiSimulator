clear,clc,close all;
addpath ../00_SourceCode
%% Initialize the solver
ori=OrigamiSolver;

%% Define geomety

% width of crease
w = 300*(10^-6);
l_j = 1.4*(10^-3);% length for joint
l_a = 7*(10^-3);% length for arm
% thickness of two layer
t1=0.2*(10^-6);
t2=0.8*(10^-6); 
plotFlag=1;
tpanel=5*(10^-6);

ori.node0=[0,      0,      0;
           0, -2*l_j,      0;
           0, -2*l_j, -2*l_j;
           0,      0, -2*l_j;
           0,  2*l_j, -2*l_j;
           0,  2*l_j,      0;
           0,  2*l_j,    l_a;
           0,      0,    l_a;
           0,      0,  2*l_a;
           0,  2*l_j,  2*l_a;]
            

% define panel
ori.panel0{1} = [1 2 3 4];
ori.panel0{2} = [1 4 5 6];
ori.panel0{3} = [1 6 7 8];
ori.panel0{4} = [7 8 9 10];
% Analyze the original pattern
ori.Mesh_AnalyzeOriginalPattern();

%% Meshing
ori.creaseWidthVec=zeros(ori.oldCreaseNum,1);
ori.creaseWidthVec(1) = w;
ori.creaseWidthVec(5) = w;
ori.creaseWidthVec(10) = w;
% ori.creaseWidthVec(9) = w;
% ori.creaseWidthVec(13) = w;
% ori.creaseWidthVec(15) = w;
% ori.creaseWidthVec(18) = w;
% ori.creaseWidthVec(21) = w;
ori.Mesh_Mesh()

ori.displayRange=30*10^(-3); % plotting range
ori.viewAngle1 = -45;
ori.viewAngle2 = 45;

% ori.newNode(7,:) = ori.newNode(7,:) - [w/2,0,0];
% ori.newNode(6,:) = ori.newNode(6,:) - [w/2,0,0];
% ori.newNode(3,:) = ori.newNode(3,:) - [w/2,0,0];
% ori.newNode(2,:) = ori.newNode(2,:) - [w/2,0,0];

% ori.Plot_UnmeshedOrigami();
% ori.Plot_MeshedOrigami();

%% Assign Mechanical Properties

ori.panelE=2*10^9; 
ori.creaseE=2*10^9; 
ori.panelPoisson=0.3;
ori.creasePoisson=0.3; 
ori.panelThickVec=ones(4,1)*tpanel;
ori.panelW=w;

% set up the diagonal rate to be large to suppress crease torsion
% ori.diagonalRate=100;
% 
ori.creaseThickVec=zeros(ori.oldCreaseNum,1);
ori.creaseThickVec(1) = t1+t2;
ori.creaseThickVec(5) = t1+t2;
ori.creaseThickVec(10) = t1+t2;

%% setup panel contact information

% ori.contactOpen=1;
% ori.ke=0.0001;
% ori.d0edge=40*(10^(-6));
% ori.d0center=40*(10^(-6));

%% Assign Thermal Properties
% 
% ori.panelThermalConductVec = [0.3;0.3;0.3;0.3;0.3;1.3]; 
% ori.creaseThermalConduct=0.3;
% ori.envThermalConduct=0.026;
% 
% % thickness of the submerged environment at RT
% ori.t2RT=(l_a+l_j)*1.5/2; 

%% Setup the loading controller

% define the self folding step
selfFold=ControllerSelfFolding;

% Assign zero strain position for creases during self-folding
% 0-2pi, This matrix can be used to manually set the zero energy rotation
% angle of the crease hinge

selfFold.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;]; % # of new node 

selfFold.increStep=40;
selfFold.tol=10^-5;
selfFold.iterMax=50;

%% Mirror robot kinemiatics 

L(1) = Link([0         0              0              pi/2]); L(1).qlim=[-pi,pi];
L(2) = Link([0         -1.4*10^-3     7*10^-3           0]); L(2).qlim=[-pi/2,3*pi/2];
L(3) = Link([0         0              0              pi/2]); L(3).qlim=[pi/2,5*pi/2];
L(4) = Link([0         7*10^-3        0              pi/2]); L(4).qlim=[-pi,pi];
L(5) = Link([0         0              0              pi/2]); L(5).qlim=[-pi,pi];
L(6) = Link([0         0              0                 0]); L(6).qlim=[-pi,pi];
myp560 = SerialLink(L,'name','PUMA560');
q2crease = [1,5,10,13,13,13];
%% Give moving Task

 q0 = [3*pi/4,-deg2rad(15), deg2rad(45),0,0,0];
 q1 = [3*pi/4, deg2rad(15),deg2rad(105),0,0,0];
 q2 = [ -pi/4, deg2rad(15),deg2rad(105),0,0,0];
 q3 = [ -pi/4,-deg2rad(15), deg2rad(45),0,0,0];

%   q0 = [3*pi/4,-deg2rad(15),deg2rad(60),0,0,0];
%  q1 = [3*pi/4, deg2rad(30),deg2rad(120),0,0,0];
%  q2 = [-pi/4,deg2rad(30),deg2rad(120),0,0,0];
%  q3 = [-pi/4,-deg2rad(15),deg2rad(60),0,0,0];
% t0 = 0;
% t1 = 2;
% t2 = t1+4;
% tf = t2+3;
t = 0:0.05:2;

pos0 = transl(myp560.fkine(q0));
pos1 = transl(myp560.fkine(q1));
pos2 = transl(myp560.fkine(q2));
pos3 = transl(myp560.fkine(q3));
% trajectory planning
This = [];
qhis = [];
qdhis = [];
qddhis = [];
poshis = [];

[T,q,qd,qdd] = mirror_robot(myp560,q0,q1,t);
pos = transl(myp560.fkine(q));
%% Multi step loading
% Here we demonstrate another way to achieve the multi-step loading using
% the "contiuingLoading" option in the package.
This = [This,T];
qhis = [qhis;q];
qdhis = [qdhis;qd];
qddhis = [qddhis;qdd];
poshis = [poshis;pos];

ori.loadingController{1}={"SelfFold",selfFold};
selfFold.targetRotZeroStrain=zeros(ori.oldCreaseNum,1);

%% visual setup
selfFold.videoOpen=0; % close the animation
selfFold.plotOpen=0; % close the plot
selfFold.targetRotZeroStrain(q2crease) = q2ang(q(1,:));
figure(3)
set(gcf,'outerposition',get(0,'screensize'),'color','w');
imgName = 'Ani_plot.gif';
pauseTime = 0.01;
hold on
maxq = max(abs(qhis),[],'all');
maxqd = max(abs(qdhis),[],'all');
maxqdd = max(abs(qddhis),[],'all');
%plot3(pos0,pos1,pos2);
%plot_sphere([pos0;pos1;pos2], 0.05, 'y');
% initiation position
ori.Solver_Solve(); 

%% trajectory tracking
ori.continuingLoading=1;
selfFold.videoOpen=1; % close the animation
for tstep = 2:length(t)
    selfFold.targetRotZeroStrain(q2crease) = q2ang(q(tstep,:));
    ori.Solver_Solve();
    hold on
    subplot(3,2,[1,3,5])
    plot_sphere(pos0, 0.0005, 'r');
    plot_sphere(pos1, 0.0005, 'r');
    plot_sphere(pos2, 0.0005, 'r');
    plot_sphere(pos3, 0.0005, 'r');
    plot3(pos(1:tstep,1),pos(1:tstep,2),pos(1:tstep,3));
    subplot(3,2,2);
    axis([0 2 -maxq maxq]);
    hold on
    title('Joint displacement');
    xlabel('Time (t/s)');
    ylabel('displacement (s/rad)')
    grid on
    plot(t(1:tstep),q(1:tstep,1),'-r',...
         t(1:tstep),q(1:tstep,2),'-g',...
         t(1:tstep),q(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    subplot(3,2,4);
    axis([0 2 -maxqd maxqd]);
    hold on
    title('Joint pesudo-velocity');
    xlabel('Time (t/s)');
    ylabel('velocity (rad/s)')
    grid on
    plot(t(1:tstep),qd(1:tstep,1),'-r',...
         t(1:tstep),qd(1:tstep,2),'-g',...
         t(1:tstep),qd(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    subplot(3,2,6);
    axis([0 2 -maxqdd maxqdd]);
    hold on
    title('Joint pesudo-acceleration');
    xlabel('Time (t/s)');
    ylabel('Acceleration (rad/s^2)')
    grid on
    plot(t(1:tstep),qdd(1:tstep,1),'-r',...
         t(1:tstep),qdd(1:tstep,2),'-g',...
         t(1:tstep),qdd(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    frame = getframe(figure(3)); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
%     if i == 1 
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
      bool = exist('./'+string(imgName),'file') ==0;
      if bool
          imwrite(imind,cm,imgName,'gif', 'Loopcount',inf); 
      else
        imwrite(imind,cm,imgName,'gif','WriteMode','append','DelayTime', pauseTime); 
     end 
end

%IC = q(end,:);
[T,q,qd,qdd] = mirror_robot(myp560,q1,pos2,t);
pos = transl(myp560.fkine(q));

% T = T(2:end);
% q = q(2:end,:);
% qd = qd(2:end,:);
% qdd = qdd(2:end,:);
This = [This,T];
qhis = [qhis;q];
qdhis = [qdhis;qd];
qddhis = [qddhis;qdd];
poshis = [poshis;pos];
maxq = max(abs(qhis),[],'all');
maxqd = max(abs(qdhis),[],'all');
maxqdd = max(abs(qddhis),[],'all');

for tstep = 1:length(t)
    selfFold.targetRotZeroStrain(q2crease) = q2ang(q(tstep,:));
    ori.Solver_Solve();
    hold on
    subplot(3,2,[1,3,5])
    plot_sphere(pos0, 0.0005, 'r');
    plot_sphere(pos1, 0.0005, 'r');
    plot_sphere(pos2, 0.0005, 'r');
    plot_sphere(pos3, 0.0005, 'r');
    plot3(poshis(1:length(t)+tstep,1),...
          poshis(1:length(t)+tstep,2),...
          poshis(1:length(t)+tstep,3));
    subplot(3,2,2);
    axis([0 4 -maxq maxq]);
    hold on
    title('Joint displacement');
    xlabel('Time (t/s)');
    ylabel('displacement (s/rad)')
    grid on
    plot(t(1:tstep)+2,q(1:tstep,1),'-r',...
         t(1:tstep)+2,q(1:tstep,2),'-g',...
         t(1:tstep)+2,q(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    subplot(3,2,4);
    axis([0 4 -maxqd maxqd]);
    hold on
    title('Joint pesudo-velocity');
    xlabel('Time (t/s)');
    ylabel('velocity (rad/s)')
    grid on
    plot(t(1:tstep)+2,qd(1:tstep,1),'-r',...
         t(1:tstep)+2,qd(1:tstep,2),'-g',...
         t(1:tstep)+2,qd(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    subplot(3,2,6);
    axis([0 4 -maxqdd maxqdd]);
    hold on
    title('Joint pesudo-acceleration');
    xlabel('Time (t/s)');
    ylabel('Acceleration (rad/s^2)')
    grid on
    plot(t(1:tstep)+2,qdd(1:tstep,1),'-r',...
         t(1:tstep)+2,qdd(1:tstep,2),'-g',...
         t(1:tstep)+2,qdd(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    frame = getframe(figure(3)); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
%     if i == 1 
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
      bool = exist('./'+string(imgName),'file') ==0;
      if bool
          imwrite(imind,cm,imgName,'gif', 'Loopcount',inf); 
      else
        imwrite(imind,cm,imgName,'gif','WriteMode','append','DelayTime', pauseTime); 
     end 
end

[T,q,qd,qdd] = mirror_robot(myp560,pos2,pos3,t);
pos = transl(myp560.fkine(q));

% T = T(2:end);
% q = q(2:end,:);
% qd = qd(2:end,:);
% qdd = qdd(2:end,:);
This = [This,T];
qhis = [qhis;q];
qdhis = [qdhis;qd];
qddhis = [qddhis;qdd];
poshis = [poshis;pos];
maxq = max(abs(qhis),[],'all');
maxqd = max(abs(qdhis),[],'all');
maxqdd = max(abs(qddhis),[],'all');

for tstep = 1:length(t)
    selfFold.targetRotZeroStrain(q2crease) = q2ang(q(tstep,:));
    ori.Solver_Solve();
    hold on
    subplot(3,2,[1,3,5])
    plot_sphere(pos0, 0.0005, 'r');
    plot_sphere(pos1, 0.0005, 'r');
    plot_sphere(pos2, 0.0005, 'r');
    plot_sphere(pos3, 0.0005, 'r');
    plot3(poshis(1:2*length(t)+tstep,1),...
          poshis(1:2*length(t)+tstep,2),...
          poshis(1:2*length(t)+tstep,3));
    subplot(3,2,2);
    axis([0 6 -maxq maxq]);
    hold on
    title('Joint displacement');
    xlabel('Time (t/s)');
    ylabel('displacement (s/rad)')
    grid on
    plot(t(1:tstep)+4,q(1:tstep,1),'-r',...
         t(1:tstep)+4,q(1:tstep,2),'-g',...
         t(1:tstep)+4,q(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    subplot(3,2,4);
    axis([0 6 -maxqd maxqd]);
    hold on
    title('Joint pesudo-velocity');
    xlabel('Time (t/s)');
    ylabel('velocity (rad/s)')
    grid on
    plot(t(1:tstep)+4,qd(1:tstep,1),'-r',...
         t(1:tstep)+4,qd(1:tstep,2),'-g',...
         t(1:tstep)+4,qd(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    subplot(3,2,6);
    axis([0 6 -maxqdd maxqdd]);
    hold on
    title('Joint pesudo-acceleration');
    xlabel('Time (t/s)');
    ylabel('Acceleration (rad/s^2)')
    grid on
    plot(t(1:tstep)+4,qdd(1:tstep,1),'-r',...
         t(1:tstep)+4,qdd(1:tstep,2),'-g',...
         t(1:tstep)+4,qdd(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    frame = getframe(figure(3)); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
%     if i == 1 
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
      bool = exist('./'+string(imgName),'file') ==0;
      if bool
          imwrite(imind,cm,imgName,'gif', 'Loopcount',inf); 
      else
        imwrite(imind,cm,imgName,'gif','WriteMode','append','DelayTime', pauseTime); 
     end 
end






