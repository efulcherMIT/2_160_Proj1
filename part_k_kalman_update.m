function [state_estimate_next, fitted_labeled_scan] = ...
    part_k_kalman_update(state_estimate_prev,scan)
%PART_K_KALMAN_UPDATE Given an a priori state estimate and a new scan, this
%function updates the state estimate, and returns the scan in its fitted
%and labeled form.

%% Unpacking the Struct
state_prev = state_estimate_prev.state;
covar_prev = state_estimate_prev.covariance;

%% Calculate the Innovation
[innovation_info, fitted_labeled_scan] = ...
    part_j_innovation_calculator(scan, state_estimate_prev);

S = innovation_info.innovation_covariance;
innov = innovation_info.innovation;
H = innovation_info.observation_matrix;

%% Calculate the Kalman Gain

K = covar_prev*H'*inv(S);

%% Calculate the State Estimate Update
%X_hat_t = X_hat_t_tm1 + K*(y - y_hat)
state_next = state_prev + K*innov;

%% Calculate the State Estimate Covariance Update
%we dont need to do the covariance propogation as it is handled by the kalman predict
covar_next = (eye(size(K*H,1)) - K*H)*covar_prev;

%% Packing the Struct
state_estimate_next.state = state_next;
state_estimate_next.covariance = covar_next;

end