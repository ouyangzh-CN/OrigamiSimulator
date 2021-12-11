function [Ts,q,qd,qdd] = mirror_robot(model,input0,input1,t,IC)
% mdl_puma560;
if nargin < 5
    IC = 0;
end
    if size(input0,2) == 6 % input is q (angle), use fkine
        q0 = input0;        
        pos0 = transl(model.fkine(q0));
    elseif size(input0,2) == 3 % input is pos (coordinates), use ikcon
        pos0 = input0;
        T0 = transl(pos0);
        %Ts = ctraj(T0,T1,length(t)); %interpolate according to T
        if IC ~= 0
            q0 = model.ikcon(T0,IC);
        else
            q0 = model.ikcon(T0);
        end
    end

    if size(input1,2) == 6
        q1 = input1;
        pos1 = transl(model.fkine(q1));
    elseif size(input1,2) == 3
        pos1 = input1;
        T1 = transl(pos1);
        if IC ~= 0
            q1 = model.ikcon(T1,IC);
        else
            q1 = model.ikcon(T1);
        end
    end

    [q,qd,qdd] = jtraj(q0,q1,t); % interpolate according to q
    Ts = model.fkine(q);
 %q = model.ikcon(Ts);
 
%figure(1)
%if exist('./jtraj.gif','file') == 0 
    %plot_sphere(pos0, 0.0015, 'y');
    %plot_sphere(pos1, 0.0015, 'y');
    %model.plot(q,'trail','b-','movie','jtraj.gif')

%end

