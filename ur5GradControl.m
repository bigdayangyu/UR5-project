function finalerr = ur5GradControl(q_start, q_target, ur5)
    % control robot by gradient descent method 
    % q_(k+1) = q_k - K*T_step*JBst'*xi
    
    addpath('interface_trframe/');
    addpath('InvKin_UR5/');
    
    v_threshold = 0.008; % meters
    w_threshold = 10/180 * pi; % radians
    K = 1;
    delta_t = 0.05; % time step
    home = [-pi/2;-pi/2;0;-pi/2;0;0]; % zero config as defined in HW6
    gt6 = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1]; % adjusts between ee_link and T frames for use with ur5InvKin
    
    % Revise end/target point to exceed by v_threshold
    g_start = ur5FwdKin(q_start - home);
    g_end = ur5FwdKin(q_target - home);
    c = g_end(1:3,4) - g_start(1:3,4); % vector between start and target positions
    c_norm = c/norm(c); % unit vector
    g_end(1:3,4) = g_end(1:3,4) + c_norm*v_threshold; % exceed the target position by v_threshold in the line direction
    q_end = ur5InvKin(g_end*gt6); % solve for the joint angles
    q_diff = abs(q_target - q_end);
    q_err = sum(q_diff);
    [~,index] = min(q_err);
    q_target = q_end(:,index); % replace q_target joint angles
    
    gdesired = ur5FwdKin(q_target - home);
    final_position = gdesired(1:3,4);
    ur5.move_joints(q_start,5);
    pause(6);
    
    % Determine initial position
    qk = ur5.get_current_joints()- home;
    gst_k = ur5FwdKin(qk);
    Jb_k = ur5BodyJacobian(qk);
    
    % Calculate error between desired and current position
    error_g = gdesired \ gst_k;
    xi_k = getXi(error_g);
    v_k = xi_k(1:3);
    w_k = xi_k(4:6);
    
    while norm(v_k) > v_threshold || norm(w_k) > w_threshold
        % Check if next position is near a singularity or exceeds working range
        qk = qk - (Jb_k' * xi_k) * K * delta_t;
        Jb_k = ur5BodyJacobian(qk);
        mu = manipulability(Jb_k,'sigmamin');
        
        % safety measure
        if abs(mu) <= 0.01
            finalerr = -1;
            disp(finalerr)
            error('workspace exceed! ');
        end
        
        if qk(2) >= 0 && qk(2) <= -pi
            error('singularity happens!');
        end
        
        % If it's safe, move to next position and compute new error
        ur5.move_joints(qk+home,1);
        pause(1.1)
        gst_k = ur5FwdKin(qk);
        Jb_k = ur5BodyJacobian(qk);
        error_g = gdesired \ gst_k;
        xi_k = getXi(error_g);
        v_k = xi_k(1:3);
        w_k = xi_k(4:6);
    end