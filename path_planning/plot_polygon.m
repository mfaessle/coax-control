function [] = plot_polygon(figurehandle,Corners,A,b,facecolor)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

set(0,'CurrentFigure',figurehandle)

n_faces = size(A,1); % number of faces
n_corn = size(Corners,1);

for i = 1:n_faces
    % find vertices of face i
    face_corner = [];
    for j = 1:n_corn
        if (abs(A(i,:)*Corners(j,:)' - b(i)) < 1e-3)
            face_corner = [face_corner; Corners(j,:)];
        end
    end
    
    % sort vertices
    n_fc = size(face_corner,1);
    com = mean(face_corner,1);
    sort_face_corner = zeros(size(face_corner));
    sort_face_corner(1,:) = face_corner(1,:);
    face_corner = face_corner(2:end,:);
    normal = A(i,:)';
    point = (sort_face_corner(1,:) - com)';
    for j = 1:n_fc-2
        n_fc_j = size(face_corner,1);
        perp = cross(normal,point);

        [~,ind] = max((face_corner-repmat(com,n_fc_j,1))*point - 1e4*((face_corner-repmat(com,n_fc_j,1))*perp < 0));
        sort_face_corner(j+1,:) = face_corner(ind,:);
        point = (sort_face_corner(j+1,:) - com)';

        face_corner = face_corner([(1:(ind-1)) ((ind+1):n_fc_j)],:);  
    end
    sort_face_corner(n_fc,:) = face_corner;
    
    % plot face
    if (sum(facecolor - [1 1 1]) == 0)
        plot3(sort_face_corner(:,1),sort_face_corner(:,2),sort_face_corner(:,3),'k--')
    else
        fill3(sort_face_corner(:,1),sort_face_corner(:,2),sort_face_corner(:,3),facecolor)
    end
end

end

