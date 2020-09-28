close all;

%% Plotting markers obtained from motion capture
% mat_file_name = "Experiment 1\Dynamic\1_1_dyn_loaded_raw.mat";
mat_file_name = "Experiment 2\test_take1_raw.mat";
mat_file_name = "Experiment 2\test_take2_raw.mat";
Data = load(mat_file_name);

N = 20;

figure('Color', 'w');

index = 1;
for i = 1:N
    P = Data.ExpData.Maker{i}.val(index, :)';
    plot3(P(1), P(2), P(3), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'b', 'MarkerSize', 5); hold on;
    
    text(P(1), P(2), P(3), num2str(i) ,'HorizontalAlignment','left','FontSize',16);
end
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 14;
xlabel_handle = xlabel('$$x$$, m', 'Interpreter', 'latex');
ylabel_handle = ylabel('$$y$$, m', 'Interpreter', 'latex');
zlabel_handle = zlabel('$$z$$, m', 'Interpreter', 'latex');
xlim = ([-2.0 2.0]);
ylim = ([-2.0 2.0]);
zlim = ([-2.0 2.0]);
axis equal;

%% Generating points for end effector cube

ee_coords = Data.ExpData.RigidBody.Position;
ee_orientation = Data.ExpData.RigidBody.quaternion;

for i = 1:3
    plot3(ee_coords(1,1),ee_coords(1,2),ee_coords(1,3),'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'b', 'MarkerSize', 5); hold on;
end

% making offsets for corners of a cube
cube_side_length = 0.1;
% 
% top points
p1 = [ee_coords(index,1)-cube_side_length/2,ee_coords(index,2)+cube_side_length/2,ee_coords(index,3)-cube_side_length/2];
p2 = [ee_coords(index,1)-cube_side_length/2,ee_coords(index,2)+cube_side_length/2,ee_coords(index,3)+cube_side_length/2];
p3 = [ee_coords(index,1)+cube_side_length/2,ee_coords(index,2)+cube_side_length/2,ee_coords(index,3)+cube_side_length/2];
p4 = [ee_coords(index,1)+cube_side_length/2,ee_coords(index,2)+cube_side_length/2,ee_coords(index,3)-cube_side_length/2];

% bottom points 
p5 = [ee_coords(index,1)-cube_side_length/2,ee_coords(index,2)-cube_side_length/2,ee_coords(index,3)-cube_side_length/2];
p6 = [ee_coords(index,1)-cube_side_length/2,ee_coords(index,2)-cube_side_length/2,ee_coords(index,3)+cube_side_length/2];
p7 = [ee_coords(index,1)+cube_side_length/2,ee_coords(index,2)-cube_side_length/2,ee_coords(index,3)+cube_side_length/2];
p8 = [ee_coords(index,1)+cube_side_length/2,ee_coords(index,2)-cube_side_length/2,ee_coords(index,3)-cube_side_length/2];

cube_points = [p1,p2,p3,p4,p5,p6,p7,p8];
%% Applying orientation on cube
% get rotation matrix for current orientation of cube
quaternion = Data.ExpData.RigidBody.quaternion(index,:);
% motive records quaternions as (x,y,z,w), in order to convert them into
% matrix matlab want (w,x,y,z) format
quat = [quaternion(end),quaternion(1:end-1)];
rot_matrix = quat2rotm(quat);

% plotting oriented version of cube
for i = 1:8
    global_index = i+20;
    point = cube_points((i-1)*3+1:(i-1)*3+3);
    point_rotated = rot_matrix * point';
    plot3(point_rotated(1), point_rotated(2), point_rotated(3), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r', 'MarkerSize', 5); hold on;
    text(point_rotated(1), point_rotated(2), point_rotated(3), num2str(global_index) ,'HorizontalAlignment','left','FontSize',16);

%     adding generated+rotated points into data table
    marker_index = i+N;
    
%     Data.ExpData.Maker{marker_index}.val(1,:) = cube_points((i-1)*3+1:(i-1)*3+3); %without rotation
    Data.ExpData.Maker{marker_index}.val(1,:) = point_rotated;
end

%% This is written after visual estimation of correspondence between regressor and mocap 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%here indexes-regressor , values - motion capture
Rods = [21,22,23,24,25,26,27,28,12,13,8,7,10,11,9,6,17,18,2,20,16,15,3,1]; %test_take_1.mat
Rods = [21,22,23,24,25,26,27,28,13,14,9,8,11,12,10,7,19,20,3,2,18,17,4,1]; %test_take_2.mat

% points that are located on the ends of rods
rods = [9,13;10,14;11,15;12,16];
% points, connected with string
string_pairs = [9,10;
                10,11;
                11,12;
                12,9;
                13,14;
                14,15;
                15,16;
                16,13;
                9,17;
                10,18;
                11,19;
                12,20;
                13,21;
                14,22;
                15,23;
                16,24;
                1,9;
                2,10;
                3,11;
                4,12;
                5,13;
                6,14;
                7,15;
                8,16               
    ];

%% Visualizing elements of stand(rods, spring, cube end-effector)
visualize = true;
if visualize
    % plotting rods
    for i=1:size(rods,1)

       P1 = Data.ExpData.Maker{Rods(rods(i,1))}.val(index, :)'; 
       P2 = Data.ExpData.Maker{Rods(rods(i,2))}.val(index, :)';
       plot3([P1(1), P2(1)], [P1(2), P2(2)], [P1(3), P2(3)], ...
            'LineWidth', 7, 'Color', [147,112,219]/255); hold on;
    end

    % plotting strings
    for i =1:size(string_pairs)
        point1_number = Rods(string_pairs(i,1));
        point2_number = Rods(string_pairs(i,2));
    %     if point1_number 
        P1 = Data.ExpData.Maker{point1_number}.val(index, :)'; 
        P2 = Data.ExpData.Maker{point2_number}.val(index, :)';
        plot3([P1(1), P2(1)], [P1(2), P2(2)], [P1(3), P2(3)], ...
            'LineWidth', 2, 'Color', [128,128,0]/255); hold on;

    end


    fill_rectangle(Data.ExpData.Maker{21}.val,Data.ExpData.Maker{22}.val,Data.ExpData.Maker{23}.val,Data.ExpData.Maker{24}.val)
    fill_rectangle(Data.ExpData.Maker{21}.val,Data.ExpData.Maker{22}.val,Data.ExpData.Maker{26}.val,Data.ExpData.Maker{25}.val)
    fill_rectangle(Data.ExpData.Maker{22}.val,Data.ExpData.Maker{23}.val,Data.ExpData.Maker{27}.val,Data.ExpData.Maker{26}.val)
    fill_rectangle(Data.ExpData.Maker{25}.val,Data.ExpData.Maker{26}.val,Data.ExpData.Maker{27}.val,Data.ExpData.Maker{28}.val)
    fill_rectangle(Data.ExpData.Maker{27}.val,Data.ExpData.Maker{23}.val,Data.ExpData.Maker{24}.val,Data.ExpData.Maker{28}.val)
    fill_rectangle(Data.ExpData.Maker{25}.val,Data.ExpData.Maker{28}.val,Data.ExpData.Maker{24}.val,Data.ExpData.Maker{21}.val)

end
%% Saving data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%this is written down after looking at the graphical output;
% Rods = [21,22,23,24,25,26,27,28,13,14,9,8,11,12,10,7,19,20,3,2,18,17,4,1];
%column #1 - node in the regressor; 
%column #2 - node in the data; 
NodeMap = [1, 21; 
           2, 22; 
           3, 23; 
           4, 24; 
           5, 25; 
           6, 26; 
           7, 27; 
           8, 28; 
           9, 12; 
           10, 13; 
           11, 8;
           12, 7;
           13, 10;
           14, 11;
           15, 9;
           16, 6;
           17, 17;
           18, 18;
           19, 2;
           20, 20;
           21, 16;
           22, 15;
           23, 3;
           24, 1
           ];


ExpData = Data.ExpData;
ExpData.Rods = Rods;
ExpData.NodeMap = NodeMap;

save(mat_file_name, 'ExpData');

mat_file_name = "Regressor\test_take1_raw.mat";
save(mat_file_name, 'ExpData');

% function for filling sides of cube
function fill_rectangle(p1,p2,p3,p4)
    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    fill3(x,y,z,[147,112,219]/255);
    hold on
end
