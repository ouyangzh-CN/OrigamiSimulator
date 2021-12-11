clear,clc,close all;
addpath ../00_SourceCode
%% Initialize the solver
ori=OrigamiSolver;

%% Define geomety

% width of crease
w = 300*(10^-6);
l_j = 1000*(10^-6);% length for joint
l_a = 7000*(10^-6);% length for arm
l_m = l_a - 2*l_j; % length for middle arm

% thickness of two layer
t1=0.2*(10^-6);
t2=0.8*(10^-6); 
plotFlag=1;
tpanel=5*(10^-6);

ori.node0=[0, 0, 0;
           l_j, 0, 0;
           0 ,0 ,l_j;
           l_j, 0 ,l_j; %node 1-4, base
           0, 0, l_j+l_a;
           l_j, 0, l_j+l_a;
           2*l_j, 0, l_j+l_a;
           0, 0, 2*l_j+l_a;
           l_j, 0, 2*l_j+l_a;
           2*l_j, 0, 2*l_j+l_a; % node 5-10, joint 1
           2*l_j+l_m, 0, 2*l_j+l_a;
           3*l_j+l_m, 0, 2*l_j+l_a;
           l_j, 0, 3*l_j+l_a;
           2*l_j, 0, 3*l_j+l_a;
           2*l_j+l_m, 0, 3*l_j+l_a;
           3*l_j+l_m, 0, 3*l_j+l_a;
           l_j*2, 0, 0;
           l_j*2, 0, l_j];

% define panel
ori.panel0{1} = [1 2 4 3];
ori.panel0{2} = [3 4 6 5];
ori.panel0{3} = [5 6 9 8];
ori.panel0{4} = [6 7 10 9];
ori.panel0{5} = [9 10 14 13];
ori.panel0{6} = [10 11 15 14];
ori.panel0{7} = [11 12 16 15];
ori.panel0{8} = [17 2 4 18];
% Analyze the original pattern
ori.Mesh_AnalyzeOriginalPattern();

%% Meshing
ori.creaseWidthVec=zeros(ori.oldCreaseNum,1);
ori.creaseWidthVec(3) = w;
ori.creaseWidthVec(4) = w;
ori.creaseWidthVec(7) = w;
ori.creaseWidthVec(9) = w;
ori.creaseWidthVec(13) = w;
ori.creaseWidthVec(15) = w;
ori.creaseWidthVec(18) = w;
ori.creaseWidthVec(21) = w;
ori.Mesh_Mesh()

ori.displayRange=10*10^(-3); % plotting range
ori.viewAngle1 = 45;
ori.viewAngle2 = 45;

% ori.newNode(7,:) = ori.newNode(7,:) - [w/2,0,0];
% ori.newNode(6,:) = ori.newNode(6,:) - [w/2,0,0];
% ori.newNode(3,:) = ori.newNode(3,:) - [w/2,0,0];
% ori.newNode(2,:) = ori.newNode(2,:) - [w/2,0,0];

ori.Plot_UnmeshedOrigami();
ori.Plot_MeshedOrigami();

%% Assign Mechanical Properties

ori.panelE=2*10^9; 
ori.creaseE=2*10^9; 
ori.panelPoisson=0.3;
ori.creasePoisson=0.3; 
ori.panelThickVec=ones(8,1)*tpanel;
ori.panelW=w;

% set up the diagonal rate to be large to suppress crease torsion
ori.diagonalRate=100;

ori.creaseThickVec=zeros(ori.oldCreaseNum,1);
ori.creaseThickVec(3) = t1+t2;
ori.creaseThickVec(4) = t1+t2;
ori.creaseThickVec(7) = t1+t2;
ori.creaseThickVec(9) = t1+t2;
ori.creaseThickVec(13) = t1+t2;
ori.creaseThickVec(15) = t1+t2;
ori.creaseThickVec(18) = t1+t2;
ori.creaseThickVec(21) = t1+t2;

%% setup panel contact information

ori.contactOpen=1;
ori.ke=0.0001;
ori.d0edge=40*(10^(-6));
ori.d0center=40*(10^(-6));

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
selfFold.targetRotZeroStrain=pi*ones(ori.oldCreaseNum,1);
selfFold.supp=[30,1,1,1;
      29,1,1,1;
      31,1,1,1;
      32,1,1,1;]; % # of new node 

selfFold.increStep=40;
selfFold.tol=10^-5;
selfFold.iterMax=50;


%% Multi step loading
% Here we demonstrate another way to achieve the multi-step loading using
% the "contiuingLoading" option in the package.

% solve the system
ori.loadingController{1}={"SelfFold",selfFold};
selfFold.targetRotZeroStrain(9)=pi+1/2*pi;
ori.Solver_Solve();

% % A second step of loading
 ori.continuingLoading=1;
 selfFold.targetRotZeroStrain(7)=pi+0.5*pi;
 ori.Solver_Solve();
% 
% % A third step of loading
 selfFold.targetRotZeroStrain(3)=pi-1/4*pi;
 selfFold.targetRotZeroStrain(4)=pi-1/4*pi;
 ori.Solver_Solve();

 selfFold.videoOpen=1;
 
 selfFold.targetRotZeroStrain(13)=pi+1/4*pi;
 ori.Solver_Solve();


