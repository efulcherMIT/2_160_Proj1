% Example code relevant to implementing Section 1: Odometry.
% Script-level code may require significant modification and expansion to 
% properly answer the questions of Section 1.

%% Part a
part_a_estimate_state_init = zeros(3,1);
part_a_estimate_state_init(1) = 1.0; %x_init (m)
part_a_estimate_state_init(2) = 5.0; %y_init (m)
part_a_estimate_state_init(3) = -4.5*pi/9; %th_init (m)
var_x = 0.25/12;
var_y = 0.25/12;
var_theta = (pi/9)^2/12;
part_a_estimate_covariance_init = [var_x,0,0;0,var_y,0;0,0,var_theta];...
    %P_init (mixed unit matrix)

%% Part b
part_b_estimate_init.state = part_a_estimate_state_init;
part_b_estimate_init.covariance = part_a_estimate_covariance_init;

part_b_time_count = odom_struct.general.time_count;

part_b_estimate_history.time = odom_struct.general.time;
part_b_estimate_history.time_count = part_b_time_count;
part_b_estimate_history.content = cell(part_b_time_count,1);

% First Time Point: No new information from data
% (just sets up first reference for delta_s)
part_b_estimate_history.content{1} = part_b_estimate_init;
wheel_prev.right = odom_struct.general.content{1}.s_right;
wheel_prev.left = odom_struct.general.content{1}.s_left;

% All other time points: new information from data, and we use it
for time_iter = 2:part_b_time_count
    % Figure out how much the wheels have turned
    wheel_next.right = odom_struct.general.content{time_iter}.s_right;
    wheel_next.left = odom_struct.general.content{time_iter}.s_left;
    wheel_turns.right = wheel_next.right - wheel_prev.right;
    wheel_turns.left = wheel_next.left - wheel_prev.left;
    wheel_prev = wheel_next;

    % Apply Kalman predict step
    part_b_estimate_history.content{time_iter} = ...
        part_b_kalman_predict(...
        part_b_estimate_history.content{time_iter-1},wheel_turns);
end

%% Part b: Post-Processing
part_b_plotting_time = part_b_estimate_history.time;
part_b_plotting_x = NaN(part_b_estimate_history.time_count,1);
part_b_plotting_y = NaN(part_b_estimate_history.time_count,1);
part_b_plotting_th = NaN(part_b_estimate_history.time_count,1);

for time_iter = 1:part_b_estimate_history.time_count
    part_b_plotting_x(time_iter) = ...
        part_b_estimate_history.content{time_iter}.state(1);
    part_b_plotting_y(time_iter) = ...
        part_b_estimate_history.content{time_iter}.state(2);
    part_b_plotting_th(time_iter) = ...
        part_b_estimate_history.content{time_iter}.state(3);
end

figure(1);clf;
plot(part_b_plotting_time,part_b_plotting_x);
xlabel('Time (s)');ylabel('x Estimate (m)');

figure(2);clf;
plot(part_b_plotting_time,part_b_plotting_y);
xlabel('Time (s)');ylabel('y Estimate (m)');

figure(3);clf;
plot(part_b_plotting_time,part_b_plotting_th);
xlabel('Time (s)');ylabel('th Estimate (rad)');

figure(4);clf;
plot(part_b_plotting_x,part_b_plotting_y);
axis equal;
xlabel('x Estimate (m)');ylabel('y Estimate (m)');

odom_plot_app(part_b_estimate_history);