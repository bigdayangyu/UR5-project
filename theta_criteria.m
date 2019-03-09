function [theta, index] = theta_criteria(get_theta, q_s1)
    % select the optimal theta
    % based on: 
    %          the norm of the previous joint angles 
    %          and the angle calculated by ur5InvKin(). i.e select the
    %          angle with smallest change compare to the previous
    %          configuration
    % input: theta calculated by ur5InvKin()
    % output: selected theta and its index in the original combination.
    q_diff = abs(get_theta - q_s1);
    q_err = sum(q_diff);
    [~,index] = min(q_err);
    theta = get_theta(:,index);
    
end