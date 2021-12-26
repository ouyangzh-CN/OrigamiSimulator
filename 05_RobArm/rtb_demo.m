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

[T,qq1,qd,qdd] = mirror_robot(myp560,q0,q1,t);
[T,qq2,qd,qdd] = mirror_robot(myp560,q1,pos2,t);
[T,qq3,qd,qdd] = mirror_robot(myp560,pos2,pos3,t);

figure(1)
hold on
    ps1 = plot_sphere(pos0, 0.0015, 'y'); set(ps1,'handlevisibility','off');
    ps2 = plot_sphere(pos1, 0.0015, 'y'); set(ps2,'handlevisibility','off');
    ps3 = plot_sphere(pos2, 0.0015, 'y'); set(ps3,'handlevisibility','off');
    ps4 = plot_sphere(pos3, 0.0015, 'y'); set(ps4,'handlevisibility','off');
myp560.plot([qq1;qq2;qq3],'trail','b-','movie','jtraj.gif')