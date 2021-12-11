clear ; clc; close all;
%       theta    d        a        alpha     offset
% 机器人各连杆DH参数
d1 = 0;
d2 = 86;
d3 = -92;
% 由于关节4为移动关节，故d4为变量，theta4为常量
theta4 = 0;

a1 = 400;
a2 = 250;
a3 = 0;
a4 = 0;

alpha1 = 0 / 180 * pi;
alpha2 = 0 / 180 * pi;
alpha3 = 180 / 180 * pi;
alpha4 = 0 / 180 * pi;
% 关节限制
q1_lim = [-180,180];   q1_lim= deg2rad(q1_lim);
q2_lim = [-180 ,180];  q2_lim= deg2rad(q2_lim);
q3_lim = [-180 ,180];  q3_lim= deg2rad(q3_lim);
q4_lim = [0 ,180];       
% 定义各个连杆，默认为转动关节
%           theta      d        a        alpha 
L(1)=Link([0         d1      a1      alpha1]); L(1).qlim=q1_lim;
L(2)=Link([0         d2      a2      alpha2]); L(2).qlim=q2_lim; L(2).offset=pi/2;
L(3)=Link([0         d3      a3      alpha3]); L(3).qlim=q3_lim;
% 移动关节需要特别指定关节类型--jointtype
L(4)=Link([theta4  0       a4      alpha4]); L(4).qlim=q4_lim; L(4).jointtype='P';
% 把上述连杆“串起来”
Scara=SerialLink(L,'name','Scara');
% 定义机器人基坐标和工具坐标的变换
Scara.base = transl(0 ,0 ,305);
Scara.tool = transl(0 ,0 ,100);
% 蒙特卡洛法求机器人工作空间
N=3000;                                               %随机次数
theta1=q1_lim(1)+diff(q1_lim)*rand(N,1); %关节1限制
theta2=q2_lim(1)+diff(q2_lim)*rand(N,1); %关节2限制
theta3=q3_lim(1)+diff(q3_lim)*rand(N,1); %关节3限制
theta4=q4_lim(1)+diff(q4_lim)*rand(N,1); %关节4限制
pos = {1,N};
% 正运动学求解
for i=1:N
    pos{i}=Scara.fkine([theta1(i) theta2(i) theta3(i) theta4(i)]);
end
% 绘图
figure
Scara.plot([0 0 0 0],'jointdiam',1)
axis equal;
hold on;
for i=1:N
    plot3(pos{i}.t(1),pos{i}.t(2),pos{i}.t(3),'r.');
    hold on;
end
grid off
view(20,30)