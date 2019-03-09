function [] = InvKinControl(q_start, q_des, ur5)
    
    addpath('interface_trframe/');
    addpath('InvKin_UR5/');
    home = [-pi/2;-pi/2;0;-pi/2;0;0];
    gt6 = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];
    
    % calculate g configuration of the staring point
    g_start = ur5FwdKin(q_start-home);
    g_des = ur5FwdKin(q_des-home);
    
    orientation1 = g_start(1:3,1:3);
    angles1 = EULERXYZINV(orientation1);
    orientation2 = g_des(1:3,1:3);
    angles2 = EULERXYZINV(orientation2);
    x_start = g_start(1:3,4);
    x_target = g_des(1:3,4);
    
    num_points = 7; % number of intermediate points for each line
    x_waypoint = linspace(x_start(1), x_target(1), num_points);
    y_waypoint = linspace(x_start(2), x_target(2), num_points);
    z_waypoint = linspace(x_start(3), x_target(3), num_points);
    
    x_rot = linspace(angles1(1), angles2(1), num_points);
    y_rot = linspace(angles1(2), angles2(2), num_points);
    z_rot = linspace(angles1(3), angles2(3), num_points);
    
    q = q_start;
    ur5.move_joints(q_start,5);
    pause(5);
    
    for i=1:num_points
        g = [EULERXYZ([x_rot(i), y_rot(i), z_rot(i)]) [x_waypoint(i) y_waypoint(i) z_waypoint(i)]';
            0 0 0 1];
        new_q = ur5InvKin(g*gt6);
        q_diff = abs(new_q - q);
        q_err = sum(q_diff);
        [~,index] = min(q_err);
        next_q = new_q(:,index);
        ur5.move_joints(next_q, 2);
        pause(2);
        q = ur5.get_current_joints();
        pause(1);
    end
    
end

