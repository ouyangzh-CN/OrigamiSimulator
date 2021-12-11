function rotation = FoldAngle(i,ori,dispacement)
% return the foldangle of ith crease

    NodeProj = [4, 3, 6, 7;
                8, 7, 10, 11;
                11,10,13, 16];

    node=squeeze(ori.newNode(NodeProj(i,:),:))'...
        +squeeze(dispacement(:,NodeProj(i,:),:))';
    
    vec1=node(:,2)-node(:,1);
    vec2=node(:,4)-node(:,3);
    
    rotation=dot(vec1,vec2)/norm(vec1)/norm(vec2);
    %rotation=sign(node(3,1)-node(3,4))*acos(rotation);
    rotation=real(acos(rotation));
end