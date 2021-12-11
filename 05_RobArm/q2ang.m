function angle = q2ang(q)
    % q2ang return the map from jointangle q to panel angle
    angle = q.*[1,1,-1,0,0,0]+[pi;pi/2;3*pi/2;0;0;0]';
end