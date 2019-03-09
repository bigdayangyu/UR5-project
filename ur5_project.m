addpath('basic function/');
addpath('interface_trframe/');
addpath('InvKin_UR5');

ur5 = ur5_interface();

s1 = 'n';
e1 = 'n';
s2 = 'n';
e2 = 'n';
keys1 = 0;
keye1 = 0;
keys2 = 0;
keye2 = 0;
keyD = 0;
keys = ["Y","y"];

%Record goal point configurations

while(keys1~=1) 
    prompt = 'Please move the end effector to the first start point... Done ? Y/N ';
    s1 = input(prompt,'s');
    keys1 = max(strcmp(s1,keys));
end

%record the 1st point
q_s1 = ur5.get_current_joints();

while(keye1~=1)
    prompt = 'Please move the end effector to the first end point... Done ? Y/N ';
    e1 = input(prompt,'s');
    keye1 = max(strcmp(e1,keys));
end
%record the 2nd point
q_e1 = ur5.get_current_joints();

while(keys2~=1)
    prompt = 'Please move the end effector to the second start point... Done ? Y/N ';
    s2 = input(prompt,'s');
    keys2 = max(strcmp(s2,keys));
end
%record the 3rd point
q_s2 = ur5.get_current_joints();

while(keye2~=1)
    prompt = 'Please move the end effector to the second end point... Done ? Y/N ';
    e2 = input(prompt,'s');
    keye2 = max(strcmp(e2,keys));
end
%record the 4th point
q_e2 = ur5.get_current_joints();

while(keyD~=1)
    prompt = 'Please give the ur5 clearance before we begin. Are you ready? Y/N ';
    D = input(prompt,'s');
    keyD = max(strcmp(D,keys));
end

% Define variables used across control methods
home = [-pi/2;-pi/2;0;-pi/2;0;0];
gt6 = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1]; % adjusts between ee_link and T frames for use with ur5InvKin
gs1 = ur5FwdKin(q_s1-home);
ge1 = ur5FwdKin(q_e1-home);
gs2 = ur5FwdKin(q_s2-home);
ge2 = ur5FwdKin(q_e2-home);

% Establish a "home" position that is close to the e1 and s2
c = (gs2(1:3,4) - ge1(1:3,4))*0.5 + [0; 0; 0.05]; % calculate the position vector between s2 and e1, find the midpoint, raise in the z direction by 5cm
home_pos = ge1(1:3,4) + c;
temp_home = gs1;
temp_home(1:3,4) = home_pos;
% Find the set of joint angles for temp_home that is closest to those for
% s1
q_temp_home = ur5InvKin(temp_home*gt6);
q_diff = abs(q_s1 - q_temp_home);
q_err = sum(q_diff);
[~,index] = min(q_err);
q_st = q_temp_home(:,index);

ge1(3,4) = ge1(3,4) + 0.05; % establishes a point above the end point of the first line
gs2(3,4) = gs2(3,4) + 0.05; % establishes a point above the start point of the second line
ge1_DH = ge1 * gt6; % convert for use with ur5InvKin
gs2_DH = gs2 * gt6; % convert for use with ur5InvKin
q_e1_above_set = ur5InvKin(ge1_DH);
q_s2_above_set = ur5InvKin(gs2_DH);

% Select the set of joint angles that results in the smallest difference
% between the offset and actual start/end points
q_diff = abs(q_e1_above_set - q_e1);
q_err = sum(q_diff);
[~,index] = min(q_err);
q_e1_above = q_e1_above_set(:,index);

q_diff = abs(q_s2_above_set - q_e1_above);
q_err = sum(q_diff);
[~,index] = min(q_err);
q_s2_above = q_s2_above_set(:,index);

ur5.move_joints(q_st, 5); % move to home position before starting
pause(3)

n = input('Press "1" for Inverse Kinematic Control. \n Press "2" for Rate Control. \n Press "3" for Gradient Control. \n Press "0" to exit.');
while (n ~= 0)
    switch n
        case 1 %Inverse Kinematic Control
            InvKinControl(q_s1,q_e1,ur5); % draw the first line

            ur5.move_joints(q_e1_above,3); % move above the end of the first line
            pause(3);
            ur5.move_joints(q_s2_above,3); % move above the start of the second line
            pause(3);
            ur5.move_joints(q_s2,3); % move to the start of the second line
            pause(3);

            InvKinControl(q_s2,q_e2,ur5); %draw the second line
            ur5.move_joints(q_st,5); % return home
            pause(5);

        case 2 %Rate Control
            ur5RateControl(q_s1,q_e1,ur5); % draw the first line
            ur5RateControl(q_e1,q_e1_above,ur5); % move above the end of the first line
            ur5RateControl(q_e1_above,q_s2_above,ur5); % move above the start of the second line
            ur5RateControl(q_s2_above,q_s2,ur5); % move to the start of the second line

%             ur5.move_joints(q_e1_above,3);
%             pause(3);
%             ur5.move_joints(q_s2_above,3);
%             pause(3);
%             ur5.move_joints(q_s2,3);
%             pause(3);

            ur5RateControl(q_s2,q_e2,ur5); % draw the second line
            ur5.move_joints(q_st,5); % return home
            pause(5);

        case 3 %Gradient Control
            ur5GradControl_2(q_s1,q_e1, 1.5, 0.05, ur5);
            ur5GradControl_2(q_e1,q_e1_above, 4, 0.05, ur5); % move above the end of the first line
            ur5GradControl_2(q_e1_above,q_s2_above, 4, 0.05, ur5); % move above the start of the second line
            ur5GradControl_2(q_s2_above,q_s2, 4, 0.05, ur5); % move to the start of the second line

%             ur5GradControl(q_s1,q_e1,ur5);
%             ur5.move_joints(q_e1_above,3);
%             pause(3);
%             ur5.move_joints(q_s2_above,3);
%             pause(3);
%             ur5.move_joints(q_s2,3);
%             pause(3);
%             ur5GradControl(q_s2,q_e2,ur5);

            ur5GradControl_2(q_s2,q_e2,1.5, 0.05, ur5);
            ur5.move_joints(q_st,5);
            pause(5);
    end
    
    n = input('Press "1" for Inverse Kinematic Control. \n Press "2" for Rate Control. \n Press "3" for Gradient Control. \n Press "0" to exit.');
end

%% Extra Credit
close all 
clc

img = imread('Avengers(3)_flipped.jpg'); %upload your image file 
grayImage = rgb2gray(img); %convert image to grayscale intensity image
binaryImage = grayImage > 220; %convert to binary (0 and 1) so that black is 1 and white is 0
% imshow(binaryImage);

boundaries = bwboundaries(binaryImage,'holes'); %search for boundaries and assign x and y coordinates

%% Convert image to xy coordinates and scale 
x_coord = [];
y_coord = [];

for k = 2:length(boundaries)
   boundary = boundaries{k};
   x_coord = [x_coord; boundary(:,2)];
   y_coord = [y_coord; boundary(:,1)];
end

% scale the image to reasonable size
x_coord = (-min(x_coord) + x_coord)/(max(x_coord)-min(x_coord))*0.10 + 0.20;
y_coord = (-min(y_coord) + y_coord)/(max(y_coord)-min(y_coord))*0.12 + 0.50;

% plot the figure
figure
scatter(x_coord, y_coord)

% hardcode in the corners of the image
a1 = [x_coord(1:50:136), y_coord(1:50:136)]; %curve
a2 = [x_coord(136:50:186), y_coord(136:50:186)];
a3 = [x_coord(186:50:612), y_coord(186:50:612)]; %curve
a4 = [x_coord(612:100:1163), y_coord(612:100:1163)];
a5 = [x_coord(1163:50:1241), y_coord(1163:50:1241)];
a6 = [x_coord(1241:100:1427), y_coord(1241:100:1427)];
a7 = [x_coord(1427:50:1527), y_coord(1427:50:1527)];
a8 = [x_coord(1527), y_coord(1527)];
a9 = [x_coord(1551:30:1657), y_coord(1551:30:1657)];
a10 = [x_coord(1657:30:1773), y_coord(1657:30:1773)];
a11 = [x_coord(1773), y_coord(1773)];
a12 = [x_coord(1795:40:1867), y_coord(1795:40:1867)];
a13 = [x_coord(1867:80:2090), y_coord(1867:80:2090)];
a14 = [x_coord(2090:45:2216), y_coord(2090:45:2216)];
a15 = [x_coord(2216:60:2318), y_coord(2216:60:2318)];
a16 = [x_coord(2318:45:2481), y_coord(2318:45:2481)];
a17 = [x_coord(2481:50:3129), y_coord(2481:50:3129)]; %curve
a18 = [x_coord(3129:50:3171), y_coord(3129:50:3171)];
a19 = [x_coord(3171:50:3968), y_coord(3171:50:3968)]; %curve
a20 = [x_coord(3968:50:4051), y_coord(3968:50:4051)];
a21 = [x_coord(4051:40:4120), y_coord(4051:40:4120)];
a22 = [x_coord(4120:50:4204), y_coord(4120:50:4204)];
a23 = [x_coord(4204:50:4625), y_coord(4204:50:4625)]; %curve
a24 = [x_coord(4625), y_coord(4625)]; %end point

%jump to triangle
b1 = [x_coord(4659:40:4716), y_coord(4659:40:4716)]; 
b2 = [x_coord(4716:40:4771), y_coord(4716:40:4771)];
b3 = [x_coord(4771), y_coord(4771)]; %end point
b4 = [x_coord(4659), y_coord(4659)];

x = [];
y = [];
for i = 1:24
    a = eval(['a' num2str(i)]);
    x = [x(1:end); a(:,1)];
    y = [y(1:end); a(:,2)];
end

x2 = [];
y2 = [];
for j = 1:4
    b = eval(['b' num2str(j)]);
    x2 = [x2(1:end); b(:,1)];
    y2 = [y2(1:end); b(:,2)];
end

figure
plot(x,y,'-o')

% x = x_coord(10:80:end);
% y = y_coord(10:80:end);

%% Establish joint angles for starting point (requires running ur5_project.m first)
% ur5.move_joints(q_s1,5);
% pause(5)
g_offset = [0,-1,0,0; 0,0,-1,0;1,0,0,0;0,0,0,1];
% q_start_z = ur5.get_current_joints();
q_start_z = q_s1;
home = [-pi/2;-pi/2;0;-pi/2;0;0];

gst_start_z = ur5FwdKin(q_start_z - home);
%gst_start = [0,-1,0,0.47;0,0,1,0.55;-1,0,0,0.12;0,0,0,1];
gst_start = gs1;
gst_start = gst_start/g_offset;
theta_start = ur5InvKin(gst_start);
[q_start, start_index] = theta_criteria(theta_start, q_s1);

%% Create joint angle set for all drawing points
% set orientation of the end effector 
R = gst_start(1:4,1:3);
p = [x(1), y(1), gst_start_z(3,4), 1]';
g =  [R, p];
theta = ur5InvKin(g);
[q, q_index] = theta_criteria(theta, q_s1);
q_store = zeros(6, length(x));

%calculate joint angle for each pose 
for i = 1:length(x)
    p = [x(i), y(i), gst_start_z(3,4), 1]';
    g =  [R, p];
    theta = ur5InvKin(g);
    q_store(:,i) = theta(:,q_index(1));
end
%calculate joint angle for each pose(triagle)
q_store2 = zeros(6, length(x2));
for i = 1:length(x2)
    p = [x2(i), y2(i), gst_start_z(3,4), 1]';
    g =  [R, p];
    theta = ur5InvKin(g);
    q_store2(:,i) = theta(:,q_index(1));
end

%% Draw the picture
ur5.move_joints(q_st, 5.5);
pause

for i = 1:length(x)
    ur5.move_joints(q_store(:,i),5);
    pause(5.5)
end

ur5.move_joints(q_st, 5.5);
pause(5.5)

for i = 1:length(x2)
    ur5.move_joints(q_store2(:,i),5);
    pause(5.5)
end

ur5.move_joints(q_st, 5.5);
pause(5.5)