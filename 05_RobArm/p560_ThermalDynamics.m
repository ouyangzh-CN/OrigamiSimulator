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
                                  
ori.Mesh_Mesh()

ori.displayRange=25*10^(-3); % plotting range
ori.viewAngle1 = -45;
ori.viewAngle2 = 45;

%  ori.Plot_UnmeshedOrigami();
%  ori.Plot_MeshedOrigami();

%% Assign Mechanical Properties

ori.panelE=2*10^9; 
ori.creaseE=2*10^9; 
ori.panelPoisson=0.3;
ori.creasePoisson=0.3; 
ori.panelThickVec=ones(4,1)*tpanel;
ori.panelW=w;

% set up the diagonal rate to be large to suppress crease torsion
ori.diagonalRate=100;
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

ori.panelThermalConductVec = [0.3;0.3;0.3;0.3]; 
ori.creaseThermalConduct=0.3;
ori.envThermalConduct=0.026;

% thickness of the submerged environment at RT
ori.t2RT=(l_a+l_j)*1.5/2; 

%% Mirror robot kinemiatics 

L(1) = Link([0         0              0              pi/2]); L(1).qlim=[-pi,pi];
L(2) = Link([0         -1.4*10^-3     7*10^-3           0]); L(2).qlim=[-pi/2,3*pi/2];
L(3) = Link([0         0              0              pi/2]); L(3).qlim=[pi/2,5*pi/2];
L(4) = Link([0         7*10^-3        0              pi/2]); L(4).qlim=[-pi,pi];
L(5) = Link([0         0              0              pi/2]); L(5).qlim=[-pi,pi];
L(6) = Link([0         0              0                 0]); L(6).qlim=[-pi,pi];
myp560 = SerialLink(L,'name','PUMA560');

t = 0:0.05:2;
 q0 = [3*pi/4,-deg2rad(15), deg2rad(45),0,0,0];
 q1 = [3*pi/4, deg2rad(15),deg2rad(105),0,0,0];
 q2 = [ -pi/4, deg2rad(15),deg2rad(105),0,0,0];
 q3 = [ -pi/4,-deg2rad(15), deg2rad(45),0,0,0];
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
Temphis = [];
LoadingHis = [];
DiffHis = [];

[T,q,qd,qdd] = mirror_robot(myp560,q0,q1,t);
pos = transl(myp560.fkine(q));

This = [This,T];
qhis = [qhis;q];
qdhis = [qdhis;qd];
qddhis = [qddhis;qdd];
poshis = [poshis;pos];

%rne = myp560.rne(q,qd,qdd);
q2crease = [1,5,10,13,13,13]; % the ith crease number
%% Proceed to the thermal loading
% we will continue the loading process

Qstep=0.15*10^-3;
maxLoadingStep=4000;

newNodeNum=size(ori.newNode);
newNodeNum=newNodeNum(1);

tempHisAssemble=zeros(newNodeNum,maxLoadingStep);
UhisAssemble=zeros(maxLoadingStep,newNodeNum,3);
loading=zeros(maxLoadingStep,3,2);
diffHis = zeros(maxLoadingStep,3);
finalQBase=0;
finalTBase=0;
finalStepBase=0;

figure(1)
%myp560.plot(q)
%% initialize jointangle
TargetAngle = zeros(ori.oldCreaseNum,1);
TargetAngle(q2crease) = q2ang(q(1,:)); % q2ang map the jointangle into panel angle


%% initialize thermal loading
    thermal=ControllerThermalLoading;
    thermal.thermalStep=1;
    thermal.tol=5*10^-7; 

    thermal.supp=[1,1,1,1;
                      2,1,1,1;
                      3,1,1,1;
                      4,1,1,1;];

    thermal.thermalBoundaryPanelVec=[];
    thermal.roomTempNode=[2;3];

    thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);
    thermal.deltaAlpha(1)=(52-14)*10^(-6);
    thermal.deltaAlpha(5)=(52-14)*10^(-6);
    thermal.deltaAlpha(10)=(52-14)*10^(-6);
    
    thermal.Emat1=0.5*79*10^9; 
    thermal.Emat2=2*10^9;
    thermal.tmat1=t1;
    thermal.tmat2=t2;

    loadingSq = [];

    
    thermal.targetCreaseHeating = loadingSq;  
    ori.loadingController{1}={"ThermalLoading",thermal};
    thermal.plotOpen = 0;
    thermal.videoOpen = 0;
    ori.Solver_Solve();    
    ori.continuingLoading=1;
    
%% initialize gesture
count = 2;
diff = 1e9;
while and(count < maxLoadingStep, vecnorm(diff)>2e-2)  % FoldAngle function get the panel angle of ith crease
        % if joint_angle ~= targetanle
        thermal=ControllerThermalLoading;
        thermal.thermalStep=1;
        thermal.tol=5*10^-7; 

        thermal.supp=[1,1,1,1;
                      2,1,1,1;
                      3,1,1,1;
                      4,1,1,1;];

        thermal.thermalBoundaryPanelVec=[];
        thermal.roomTempNode=[2;3];

        thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);
        thermal.deltaAlpha(1)=(52-14)*10^(-6);
        thermal.deltaAlpha(5)=(52-14)*10^(-6);
        thermal.deltaAlpha(10)=(52-14)*10^(-6);
    
        thermal.Emat1=0.5*79*10^9; 
        thermal.Emat2=2*10^9;
        thermal.tmat1=t1;
        thermal.tmat2=t2;

        loadingSq = [];

        diff = ori.currentRotZeroStrain(q2crease(1:3)) - TargetAngle(q2crease(1:3));
        absdiff = abs(diff);
        minstep = max(absdiff);
        for i = 1:3
            if and(absdiff(i) < minstep, absdiff(i)>1e-2)
                minstep = absdiff(i);
            end
        end
    for i = 1:3 % 3 joint
        %foldAngle = FoldAngle(i,ori,UhisAssemble(count-1,:,:));
        dq = 0;
        if abs(diff(i)) > 1e-2
            pm = -sign(diff(i)); % add loading or decrease loading
            dq = pm*Qstep*abs(diff(i)/minstep);
        end
        loadingSq = [loadingSq;q2crease(i),dq];
    end
        thermal.targetCreaseHeating = loadingSq;  
        ori.loadingController{1}={"ThermalLoading",thermal};
        thermal.plotOpen = 0;
        thermal.videoOpen = 0;
        ori.Solver_Solve();    
        ori.continuingLoading=1;
        
        %UhisAssemble(count,:,:)=thermal.Uhis(1,:,:);
        %tempHisAssemble(:,count)=thermal.temperatureHis(:,1);
        % Solve the fold angle for stop loading
        count = count + 1;
end

UhisAssemble(1,:,:)=thermal.Uhis(1,:,:);
tempHisAssemble(:,1)=thermal.temperatureHis(:,1);

figure(3)
set(gcf,'outerposition',get(0,'screensize'),'color','w');
imgName = 'Ani_thermalplot.gif';
pauseTime = 0.01;
hold on
maxq = max(abs(qhis),[],'all');
maxqd = max(abs(qdhis),[],'all');
maxqdd = max(abs(qddhis),[],'all');

%% trajectory tracking
ori.continuingLoading=1;
thermal.videoOpen=1; 
for tstep = 1:length(t)
    count = 1;
    diff = 1e9;
    TargetAngle(q2crease) = q2ang(q(tstep,:));
    loadingSq_step = zeros(3,2);
    while and(count < maxLoadingStep, vecnorm(diff)>2e-2)  % FoldAngle function get the panel angle of ith crease
        % if joint_angle ~= targetanle
        thermal=ControllerThermalLoading;
        thermal.thermalStep=1;
        thermal.tol=5*10^-7; 

        thermal.supp=[1,1,1,1;
                      2,1,1,1;
                      3,1,1,1;
                      4,1,1,1;];

        thermal.thermalBoundaryPanelVec=[];
        thermal.roomTempNode=[2;3];

        thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);
        thermal.deltaAlpha(1)=(52-14)*10^(-6);
        thermal.deltaAlpha(5)=(52-14)*10^(-6);
        thermal.deltaAlpha(10)=(52-14)*10^(-6);
    
        thermal.Emat1=0.5*79*10^9; 
        thermal.Emat2=2*10^9;
        thermal.tmat1=t1;
        thermal.tmat2=t2;

        loadingSq = [];

        diff = ori.currentRotZeroStrain(q2crease(1:3)) - TargetAngle(q2crease(1:3));
                absdiff = abs(diff);
        minstep = max(absdiff);
        for i = 1:3
            if and(absdiff(i) < minstep, absdiff(i)>1e-2)
                minstep = absdiff(i);
            end
        end
    for i = 1:3 % 3 joint
        %foldAngle = FoldAngle(i,ori,UhisAssemble(count-1,:,:));
        dq = 0;
        if abs(diff(i)) > 1e-2
            pm = -sign(diff(i)); % add loading or decrease loading
            dq = pm*Qstep*abs(diff(i)/minstep);
        end
        loadingSq = [loadingSq;q2crease(i),dq];
    end
        thermal.targetCreaseHeating = loadingSq;  
        ori.loadingController{1}={"ThermalLoading",thermal};
        thermal.plotOpen = 0;
        thermal.videoOpen = 1;
        
        ori.Solver_Solve();    
        ori.continuingLoading=1;
        
        % Solve the fold angle for stop loading
        count = count + 1;
        loadingSq_step = loadingSq_step + loadingSq;
    end

    UhisAssemble(tstep,:,:)=thermal.Uhis(1,:,:);
    tempHisAssemble(:,tstep)=thermal.temperatureHis(:,1);
    loading(tstep,:,:) = loadingSq_step;
    diffHis(tstep,:) = diff;
    hold on
    %subplot(3,2,[1,3,5])
    ps1 = plot_sphere(pos0, 0.0005, 'r'); set(ps1,'handlevisibility','off');
    ps2 = plot_sphere(pos1, 0.0005, 'r'); set(ps2,'handlevisibility','off');
    ps3 = plot_sphere(pos2, 0.0005, 'r'); set(ps3,'handlevisibility','off');
    ps4 = plot_sphere(pos3, 0.0005, 'r'); set(ps4,'handlevisibility','off');
    traj = plot3(pos(1:tstep,1),pos(1:tstep,2),pos(1:tstep,3),'-b');
    subplot(3,2,2);
    axis([0 2 -1.1*maxq 1.1*maxq]);
    hold on
    title('Joint displacement');
    xlabel('Time (t/s)');
    ylabel('displacement (rad)')
    grid on
    plot(t(1:tstep),q(1:tstep,1),'-r',...
         t(1:tstep),q(1:tstep,2),'-g',...
         t(1:tstep),q(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')

    subplot(3,2,4);
    axis([0 2 ...
        1.1*min(diffHis(1:tstep,:),[],'all') ...
        1.1*max(diffHis(1:tstep,:),[],'all')]);
    hold on
    title('tracking bias');
    xlabel('Time (t/s)');
    ylabel('displacement (rad)')
    grid on
    plot(t(1:tstep),diffHis(1:tstep,1),'-r',...
         t(1:tstep),diffHis(1:tstep,2),'-g',...
         t(1:tstep),diffHis(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    hold on

    subplot(3,2,6);
    grid on
    hold on
    axis([0 2 ...
          1.1*min(tempHisAssemble(:,1:tstep),[],'all') ...
          1.1*max(tempHisAssemble(:,1:tstep),[],'all')])
    plot(t(1:tstep),tempHisAssemble(19,1:tstep),'-r',...
         t(1:tstep),tempHisAssemble(22,1:tstep),'-g',...
         t(1:tstep),tempHisAssemble(25,1:tstep),'-b');
%     plotyyy(t(1:tstep),tempHisAssemble(19,1:tstep),...
%             t(1:tstep),tempHisAssemble(22,1:tstep),...
%             t(1:tstep),tempHisAssemble(25,1:tstep));
    %axis([0 2 -maxqd maxqd]);
    title('Node Temperature');
    xlabel('Time (t/s)');
    ylabel('Degree C')
    legend('joint1-Node19','joint2-Node22','joint3-Node25')
    frame = getframe(figure(3)); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    bool = exist('./'+string(imgName),'file') ==0;
      if bool
          imwrite(imind,cm,imgName,'gif', 'Loopcount',inf); 
      else
        imwrite(imind,cm,imgName,'gif','WriteMode','append','DelayTime', pauseTime); 
     end 
end

[T,q,qd,qdd] = mirror_robot(myp560,q1,pos2,t);
pos = transl(myp560.fkine(q));

Temphis = [Temphis;tempHisAssemble];
LoadingHis = [LoadingHis;loading];
DiffHis = [DiffHis;diffHis];
This = [This,T];
qhis = [qhis;q];
qdhis = [qdhis;qd];
qddhis = [qddhis;qdd];
poshis = [poshis;pos];
maxq = max(abs(qhis),[],'all');
maxqd = max(abs(qdhis),[],'all');
maxqdd = max(abs(qddhis),[],'all');

for tstep = 1:length(t)
    count = 1;
    diff = 1e9;
    TargetAngle(q2crease) = q2ang(q(tstep,:));
    loadingSq_step = zeros(3,2);
    while and(count < maxLoadingStep, vecnorm(diff)>2e-2)  % FoldAngle function get the panel angle of ith crease
        % if joint_angle ~= targetanle
        thermal=ControllerThermalLoading;
        thermal.thermalStep=1;
        thermal.tol=5*10^-7; 

        thermal.supp=[1,1,1,1;
                      2,1,1,1;
                      3,1,1,1;
                      4,1,1,1;];

        thermal.thermalBoundaryPanelVec=[];
        thermal.roomTempNode=[2;3];

        thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);
        thermal.deltaAlpha(1)=(52-14)*10^(-6);
        thermal.deltaAlpha(5)=(52-14)*10^(-6);
        thermal.deltaAlpha(10)=(52-14)*10^(-6);
    
        thermal.Emat1=0.5*79*10^9; 
        thermal.Emat2=2*10^9;
        thermal.tmat1=t1;
        thermal.tmat2=t2;

        loadingSq = [];

        diff = ori.currentRotZeroStrain(q2crease(1:3)) - TargetAngle(q2crease(1:3));
                absdiff = abs(diff);
        minstep = max(absdiff);
        for i = 1:3
            if and(absdiff(i) < minstep, absdiff(i)>1e-2)
                minstep = absdiff(i);
            end
        end
    for i = 1:3 % 3 joint
        %foldAngle = FoldAngle(i,ori,UhisAssemble(count-1,:,:));
        dq = 0;
        if abs(diff(i)) > 1e-2
            pm = -sign(diff(i)); % add loading or decrease loading
            dq = pm*Qstep*abs(diff(i)/minstep);
        end
        loadingSq = [loadingSq;q2crease(i),dq];
    end
        thermal.targetCreaseHeating = loadingSq;  
        ori.loadingController{1}={"ThermalLoading",thermal};
        thermal.plotOpen = 0;
        thermal.videoOpen = 1;
        
        ori.Solver_Solve();    
        ori.continuingLoading=1;
        
        % Solve the fold angle for stop loading
        count = count + 1;
        loadingSq_step = loadingSq_step + loadingSq;
    end

    UhisAssemble(tstep,:,:)=thermal.Uhis(1,:,:);
    tempHisAssemble(:,tstep)=thermal.temperatureHis(:,1);
    loading(tstep,:,:) = loadingSq_step;
    diffHis(tstep,:) = diff;
    hold on
    %subplot(3,2,[1,3,5])
    ps1 = plot_sphere(pos0, 0.0005, 'r'); set(ps1,'handlevisibility','off');
    ps2 = plot_sphere(pos1, 0.0005, 'r'); set(ps2,'handlevisibility','off');
    ps3 = plot_sphere(pos2, 0.0005, 'r'); set(ps3,'handlevisibility','off');
    ps4 = plot_sphere(pos3, 0.0005, 'r'); set(ps4,'handlevisibility','off');
    traj = plot3(poshis(1:length(t)+tstep,1),...
                 poshis(1:length(t)+tstep,2),...
                 poshis(1:length(t)+tstep,3),'-b');
    subplot(3,2,2);
    axis([0 4 -1.1*maxq 1.1*maxq]);
    hold on
    title('Joint displacement');
    xlabel('Time (t/s)');
    ylabel('displacement (rad)')
    grid on
    plot(t(1:tstep)+2,q(1:tstep,1),'-r',...
         t(1:tstep)+2,q(1:tstep,2),'-g',...
         t(1:tstep)+2,q(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')

    subplot(3,2,4);
    axis([0 4 ...
        1.1*min(diffHis(1:length(t)+tstep,:),[],'all') ...
        1.1*max(diffHis(1:length(t)+tstep,:),[],'all')]);
    hold on
    title('tracking bias');
    xlabel('Time (t/s)');
    ylabel('displacement (rad)')
    grid on
    plot(t(1:tstep)+2,diffHis(1:tstep,1),'-r',...
         t(1:tstep)+2,diffHis(1:tstep,2),'-g',...
         t(1:tstep)+2,diffHis(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    hold on

    subplot(3,2,6);
    grid on
    hold on
    axis([0 4 ...
          1.1*min(Temphis(:,1:length(t)+tstep),[],'all') ...
          1.1*max(Temphis(:,1:length(t)+tstep),[],'all')])
    plot(t(1:tstep)+2,tempHisAssemble(19,1:tstep),'-r',...
         t(1:tstep)+2,tempHisAssemble(22,1:tstep),'-g',...
         t(1:tstep)+2,tempHisAssemble(25,1:tstep),'-b');
%     plotyyy(t(1:tstep),tempHisAssemble(19,1:tstep),...
%             t(1:tstep),tempHisAssemble(22,1:tstep),...
%             t(1:tstep),tempHisAssemble(25,1:tstep));
    %axis([0 2 -maxqd maxqd]);
    title('Node Temperature');
    xlabel('Time (t/s)');
    ylabel('Degree C')
    legend('joint1-Node19','joint2-Node22','joint3-Node25')
    frame = getframe(figure(3)); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    bool = exist('./'+string(imgName),'file') ==0;
      if bool
          imwrite(imind,cm,imgName,'gif', 'Loopcount',inf); 
      else
        imwrite(imind,cm,imgName,'gif','WriteMode','append','DelayTime', pauseTime); 
     end 
end

[T,q,qd,qdd] = mirror_robot(myp560,pos2,pos3,t);
pos = transl(myp560.fkine(q));

Temphis = [Temphis;tempHisAssemble];
LoadingHis = [LoadingHis;loading];
DiffHis = [DiffHis;diffHis];
This = [This,T];
qhis = [qhis;q];
qdhis = [qdhis;qd];
qddhis = [qddhis;qdd];
poshis = [poshis;pos];
maxq = max(abs(qhis),[],'all');
maxqd = max(abs(qdhis),[],'all');
maxqdd = max(abs(qddhis),[],'all');

for tstep = 1:length(t)
    count = 1;
    diff = 1e9;
    TargetAngle(q2crease) = q2ang(q(tstep,:));
    loadingSq_step = zeros(3,2);
    while and(count < maxLoadingStep, vecnorm(diff)>2e-2)  % FoldAngle function get the panel angle of ith crease
        % if joint_angle ~= targetanle
        thermal=ControllerThermalLoading;
        thermal.thermalStep=1;
        thermal.tol=5*10^-7; 

        thermal.supp=[1,1,1,1;
                      2,1,1,1;
                      3,1,1,1;
                      4,1,1,1;];

        thermal.thermalBoundaryPanelVec=[];
        thermal.roomTempNode=[2;3];

        thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);
        thermal.deltaAlpha(1)=(52-14)*10^(-6);
        thermal.deltaAlpha(5)=(52-14)*10^(-6);
        thermal.deltaAlpha(10)=(52-14)*10^(-6);
    
        thermal.Emat1=0.5*79*10^9; 
        thermal.Emat2=2*10^9;
        thermal.tmat1=t1;
        thermal.tmat2=t2;

        loadingSq = [];

        diff = ori.currentRotZeroStrain(q2crease(1:3)) - TargetAngle(q2crease(1:3));
                absdiff = abs(diff);
        minstep = max(absdiff);
        for i = 1:3
            if and(absdiff(i) < minstep, absdiff(i)>1e-2)
                minstep = absdiff(i);
            end
        end
    for i = 1:3 % 3 joint
        %foldAngle = FoldAngle(i,ori,UhisAssemble(count-1,:,:));
        dq = 0;
        if abs(diff(i)) > 1e-2
            pm = -sign(diff(i)); % add loading or decrease loading
            dq = pm*Qstep*abs(diff(i)/minstep);
        end
        loadingSq = [loadingSq;q2crease(i),dq];
    end
        thermal.targetCreaseHeating = loadingSq;  
        ori.loadingController{1}={"ThermalLoading",thermal};
        thermal.plotOpen = 0;
        thermal.videoOpen = 1;
        
        ori.Solver_Solve();    
        ori.continuingLoading=1;
        
        % Solve the fold angle for stop loading
        count = count + 1;
        loadingSq_step = loadingSq_step + loadingSq;
    end

    UhisAssemble(tstep,:,:)=thermal.Uhis(1,:,:);
    tempHisAssemble(:,tstep)=thermal.temperatureHis(:,1);
    loading(tstep,:,:) = loadingSq_step;
    diffHis(tstep,:) = diff;
    hold on
    %subplot(3,2,[1,3,5])
    ps1 = plot_sphere(pos0, 0.0005, 'r'); set(ps1,'handlevisibility','off');
    ps2 = plot_sphere(pos1, 0.0005, 'r'); set(ps2,'handlevisibility','off');
    ps3 = plot_sphere(pos2, 0.0005, 'r'); set(ps3,'handlevisibility','off');
    ps4 = plot_sphere(pos3, 0.0005, 'r'); set(ps4,'handlevisibility','off');
    traj = plot3(poshis(1:2*length(t)+tstep,1),...
                 poshis(1:2*length(t)+tstep,2),...
                 poshis(1:2*length(t)+tstep,3),'-b');
    subplot(3,2,2);
    axis([0 6 -1.1*maxq 1.1*maxq]);
    hold on
    title('Joint displacement');
    xlabel('Time (t/s)');
    ylabel('displacement (rad)')
    grid on
    plot(t(1:tstep)+4,q(1:tstep,1),'-r',...
         t(1:tstep)+4,q(1:tstep,2),'-g',...
         t(1:tstep)+4,q(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')

    subplot(3,2,4);
    axis([0 6 ...
        1.1*min(diffHis(1:2*length(t)+tstep,:),[],'all') ...
        1.1*max(diffHis(1:2*length(t)+tstep,:),[],'all')]);
    hold on
    title('tracking bias');
    xlabel('Time (t/s)');
    ylabel('displacement (rad)')
    grid on
    plot(t(1:tstep)+4,diffHis(1:tstep,1),'-r',...
         t(1:tstep)+4,diffHis(1:tstep,2),'-g',...
         t(1:tstep)+4,diffHis(1:tstep,3),'-b');
    legend('joint1','joint2','joint3')
    hold on

    subplot(3,2,6);
    grid on
    hold on
    axis([0 6 ...
          1.1*min(Temphis(:,1:2*length(t)+tstep),[],'all') ...
          1.1*max(Temphis(:,1:2*length(t)+tstep),[],'all')])
    plot(t(1:tstep)+4,tempHisAssemble(19,1:tstep),'-r',...
         t(1:tstep)+4,tempHisAssemble(22,1:tstep),'-g',...
         t(1:tstep)+4,tempHisAssemble(25,1:tstep),'-b');
%     plotyyy(t(1:tstep),tempHisAssemble(19,1:tstep),...
%             t(1:tstep),tempHisAssemble(22,1:tstep),...
%             t(1:tstep),tempHisAssemble(25,1:tstep));
    %axis([0 2 -maxqd maxqd]);
    title('Node Temperature');
    xlabel('Time (t/s)');
    ylabel('Degree C')
    legend('joint1-Node19','joint2-Node22','joint3-Node25')
    frame = getframe(figure(3)); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    bool = exist('./'+string(imgName),'file') ==0;
      if bool
          imwrite(imind,cm,imgName,'gif', 'Loopcount',inf); 
      else
        imwrite(imind,cm,imgName,'gif','WriteMode','append','DelayTime', pauseTime); 
     end 
end




