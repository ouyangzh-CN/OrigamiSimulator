%% plot the deformation history of the simulated results.
%
% This funciton plots the deformation history of the simlation results,
% with an assemble (self-folding) process and another loading process. The
% function also generates a GIF animation for inspection.
%
% Input: 
%       viewControl: general plotting set up;
%       newNode: the nodal coordinates;
%       newPanel: the nodes of each panel;
%       UhisLoading: the deformation history for loading;
%       UhisAssemble: the deformation history for self-assemble.
%

function Plot_DeformedHis(obj,undeformedNode,UhisLoading)

View1=obj.viewAngle1;
View2=obj.viewAngle2;
Vsize=obj.displayRange;
Vratio=obj.displayRangeRatio;

pauseTime=0.01;
filename='OriAnimation.gif';
bool = exist('./'+string(filename),'file') ==0;
h=figure(3);

% view(View1,View2); 
% set(gca,'DataAspectRatio',[1 1 1])
% axis([-0.2*Vsize Vsize -0.2*Vsize Vsize -0.2*Vsize Vsize])
% axis([-Vsize Vsize -Vsize Vsize -Vsize Vsize])

B=size(obj.newPanel);
FaceNum=B(2);


B=size(UhisLoading);
Incre=B(1);
n1=B(2);
n2=B(3);

set(gcf, 'color', 'white');
%set(gcf,'Position',[0,-300,1000,1000])
    

for i=1:Incre:Incre
    cla(subplot(3,2,[1,3,5]))
    subplot(3,2,[1,3,5])
    view(View1,View2); 
    set(gca,'DataAspectRatio',[1 1 1])
    axis([-Vratio*Vsize Vsize -Vratio*Vsize Vsize -Vratio*Vsize Vsize])
    tempU = squeeze(UhisLoading(i,1:n1,1:n2));

    deformNode=undeformedNode+tempU;
    for j=1:FaceNum
        tempPanel=cell2mat(obj.newPanel(j));
        patch('Vertices',deformNode,'Faces',tempPanel,...
            'FaceColor',obj.faceColorAnimation, ...
            'FaceAlpha', obj.faceAlphaAnimation);     
    end
    %pause(pauseTime);  
    
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
%     if i == 1 
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
      if bool
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', pauseTime); 
     end 
end
