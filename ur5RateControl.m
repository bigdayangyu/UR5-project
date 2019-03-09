function finalerr =  ur5RateControl(q_start, q_target, ur5)
    % Iteratively implements a discrete-time resolved rate control system
    % q_(k+1) = q_k - K*T_step*inv(JBst)*xi
    % Note that gst0 is defined using the zero configuration in HW6, [-pi/2; -pi/2; 0; -pi/2; 0; 0]
    % Takes inputs gdesired (desired end effector pose, in cm), K (controller gain),
    % and ur5 object and outputs the final positional error in cm (or -1 if there is a failure)
    addpath('interface_trframe/');
    addpath('InvKin_UR5/');
    
    
    v_threshold = 0.004; % meters
    w_threshold = 5/180 * pi; % radians
    K = 10;
    delta_t = 0.01; % time step
    home = [-pi/2;-pi/2;0;-pi/2;0;0]; % zero config as defined in HW6
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
        qk = qk - (Jb_k \ xi_k) * K * delta_t;
        Jb_k = ur5BodyJacobian(qk);
        mu = manipulability(Jb_k,'sigmamin');
        
        if abs(mu) <= 0.01
            finalerr = -1;
            disp(finalerr)
            error('singularity happens!');
        end
        
         if qk(2) >= 0 && qk(2) <= -pi
            finalerr = -1;
            disp(finalerr)
            error('exceeds working range');
        end
        
        % If it's safe, move to next position and compute new error
        ur5.move_joints(qk+home,2);
        pause(2);
        gst_k = ur5FwdKin(qk);
        Jb_k = ur5BodyJacobian(qk);
        
        error_g = gdesired \ gst_k;
        xi_k = getXi(error_g);
        v_k = xi_k(1:3);
        w_k = xi_k(4:6);
    end