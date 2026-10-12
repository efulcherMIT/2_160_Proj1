function state_estimate_next = ...
    part_b_kalman_predict(state_estimate_prev,wheel_turns)
%PART_B_KALMAN_PREDICT Simulates the vehicle moving with certain wheel
%turns, updating the vehicle pose and pose covariance.

%% Unpacking the Structs
% Previous State Estimate
state_t = state_estimate_prev.state;%(mixed-unit vector)
P_t = state_estimate_prev.covariance;%(mixed-unit matrix)
state_dim = length(state_t); %(count)
x_t = state_t(1); %(m)
y_t = state_t(2); %(m)
th_t = state_t(3); %(rad)

% Wheel Turns
ds_r = wheel_turns.right; %(m)
ds_l = wheel_turns.left; %(m)

%% Define System Parameters
b = 0.3; %(m) wheel distance
k_r = 0.04; %(m) right wheel process noise variance factor
k_l = 0.04; %(m) left wheel process noise variance factor

%% State Estimate Simulation
v_t = 1/2*(ds_l+ds_r);%(m)
dth = 1/b*(ds_r-ds_l);%(rad)

x_tp1 = x_t+v_t*cos(th_t+1/2*dth);
y_tp1 = y_t + v_t*sin(th_t+1/2*dth);
th_tp1 = th_t + dth;

%% Covariance Propagation
%helper vars
c_theta = cos(th_t+1/2*dth);
s_theta = sin(th_t+1/2*dth);

A = [1,0,-v_t*s_theta;
    0,1,v_t*c_theta;
    0,0,1];

G = [1/2*c_theta - v_t/b*s_theta, 1/2*c_theta + v_t/b*s_theta;
    1/2*s_theta + v_t/b*c_theta, 1/2*s_theta - v_t/b*c_theta;
    1/b,-1/b];

Sigma_S = [k_r*abs(ds_r),0;0,k_l*abs(ds_l)]; %covariance of ds_r and ds_l


% propogate error cov, as we dont have measurements, we dont do kalman updating or anytthing like that
P_tp1 = A*P_t*A' + G*Sigma_S*G';

%% Repacking the Structs
state_tp1 = state_t;
state_tp1(1) = x_tp1;
state_tp1(2) = y_tp1;
state_tp1(3) = th_tp1;

state_estimate_next.state = state_tp1;
state_estimate_next.covariance = P_tp1;

end