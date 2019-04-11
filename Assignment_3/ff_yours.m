%%%% Dynamics Controller
function tau_ff = ff_yours(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, your_parameters, rp)
%     the robot draws the ellipse approx. twice, consider wrapping the
%     angles to get better training data
%     
%     if you want to use Cartesian positions instead of joint positions in
%     your function approximator:
%     [x_des, x_d_des, x_dd_des, ~] = FK(th_des, th_d_des, th_dd_des, rp);
%     and this is the only purpose for which you are allowed to use the
%     robot parameters rp.
    
    th_mean = (th_curr+th_des)*0.5;
    th_d_mean = (th_d_curr+th_d_des)*0.5;

    
    [M_mean, V_mean, G_mean] = RBD_matrices(th_mean,rp);
    tau_ff = M_mean*(th_dd_des) + V_mean*(th_d_mean.^2) + G_mean;
end