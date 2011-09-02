filename = 'CoaxCalib.xml';

cog_position = [20 0 113.5]; % pos of coax CoG in calibration board frame

Markers_xy = [-331.546 -155.616 0;
              -198.045 -187.147 0;
              220.145 -187.147 0;
              356.643 -169.434 0;
              295.533 105.415 0;
              356.643 179.023 0;
              246.006 202.198 0;
              -175.810 167.579 0;
              -313.705 188.863 0;
              -356.002 92.593 0];

Markers(:,1) = Markers_xy(:,1) - cog_position(1);
Markers(:,2) = Markers_xy(:,2) - cog_position(2);
Markers(:,3) = Markers_xy(:,3) - cog_position(3);

MarkerNames = {'CoaxCalib:Unlabeled67007',
               'CoaxCalib:Unlabeled67012',
               'CoaxCalib:Unlabeled67003',
               'CoaxCalib:Unlabeled67009',
               'CoaxCalib:Unlabeled67005',
               'CoaxCalib:Unlabeled67006',
               'CoaxCalib:Unlabeled67010',
               'CoaxCalib:Unlabeled67011',
               'CoaxCalib:Unlabeled67008',
               'CoaxCalib:Unlabeled67004'};

delete(filename)

%% compose string
my_string{1,1} = '<reference>';
my_string{2,1} = '    <markers>';
for i=3 : 4*size(Markers,1)+2
    if (mod(i-2,4) == 1)
        my_string{i,1} = '        <marker>';
    elseif (mod(i-2,4) == 2)
        my_string{i,1} = horzcat('            <name>',MarkerNames{i/4},'</name>');
    elseif (mod(i-2,4) == 3)
        my_string{i,1} = horzcat('            <position>',num2str(Markers(((i-1)/4),:),'%3.3f %3.3f %3.3f'),'</position>');
    else
        my_string{i,1} = '        </marker>';
    end
    
end
my_string{4*size(Markers,1)+3,1} = '    </markers>';
my_string{4*size(Markers,1)+4,1} = '</reference>';


%% write file
fid = fopen(filename,'wt');
n = size(my_string,1);
for j = 1:n
    if (~isempty(my_string{j,1}))
        if (j~=n)
            fprintf(fid,'%s \n',my_string{j,1});
        else
            fprintf(fid,'%s',my_string{j,1});
        end
    end
end
fclose(fid);
