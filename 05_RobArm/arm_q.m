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

ori.displayRange=20*10^(-3); % plotting range
ori.viewAngle1 = 45;
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

% Assign Thermal Properties

ori.panelThermalConductVec = [0.3;0.3;0.3;0.3]; 
ori.creaseThermalConduct=0.3;
ori.envThermalConduct=0.026;

% thickness of the submerged environment at RT
ori.t2RT=(l_a+l_j)*1.5/2; 

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

%% Mirror robot kinemiatics offset
L(1) = Link([0         0              0              pi/2]); L(1).qlim=[0,2*pi];
L(2) = Link([0         -1.4*10^-3     7*10^-3           0]); L(2).qlim=[-pi/2,3*pi/2];
L(3) = Link([0         0              0              pi/2]); L(3).qlim=[pi/2,5*pi/2];
L(4) = Link([0         7*10^-3        0              pi/2]); L(4).qlim=[-pi,pi];
L(5) = Link([0         0              0              pi/2]); L(5).qlim=[-pi,pi];
L(6) = Link([0         0              0                 0]); L(6).qlim=[-pi,pi];
myp560 = SerialLink(L,'name','PUMA560');

t = 0:0.05:2;

% pos0 = [-0.5,0.5,-0.5]*l_a;
% pos1 = [-0.5,0.5,0.3]*l_a;
pos0 = transl(myp560.fkine([0,pi/2,0,0,0,0]));
pos1 = transl(myp560.fkine([pi/2,0,0,0,0,0]));
[T,q] = mirror_robot(myp560,pos0,pos1,t);
%% Multi step loading
% Here we demonstrate another way to achieve the multi-step loading using
% the "contiuingLoading" option in the package.

q2crease = [1,5,10,13,13,13];

Qstep=0.1*10^-3;
maxLoadingStep=400;

newNodeNum=size(ori.newNode);
newNodeNum=newNodeNum(1);

tempHisAssemble=zeros(newNodeNum,maxLoadingStep);
UhisAssemble=zeros(maxLoadingStep,newNodeNum,3);

finalQBase=0;
finalTBase=0;
finalStepBase=0;

ori.loadingController{1}={"SelfFold",selfFold};
selfFold.targetRotZeroStrain=zeros(ori.oldCreaseNum,1);

% initiation position
% ori.Solver_Solve();
selfFold.videoOpen=1; % close the animation
selfFold.plotOpen=0; % close the plot
selfFold.targetRotZeroStrain(q2crease) = q2ang(q(1,:));
ori.Solver_Solve(); 

ori.continuingLoading=1;
for tstep = 2:length(t)
    selfFold.targetRotZeroStrain(q2crease) = q2ang(q(tstep,:));
    ori.Solver_Solve();
end

% % A third step of loading



