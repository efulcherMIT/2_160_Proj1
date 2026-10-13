clc;
clear all;
%% WALL 1
fprintf("\nWALL 1\n")
x_p1 = 0;
y_p1 = 0;
x_p2 = 5;
y_p2 = 0;
syms alpha r
sols = solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 2
fprintf("\nWALL 2\n")
x_p1 = 5;
y_p1 = 0;
x_p2 = 5;
y_p2 = 4;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 3
fprintf("\nWALL 3\n")
x_p1 = 5;
y_p1 = 4;
x_p2 = 2;
y_p2 = 4;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 4
fprintf("\nWALL 4\n")
x_p1 = 2;
y_p1 = 4;
x_p2 = 2;
y_p2 = 6;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 5
fprintf("\nWALL 5\n")
x_p1 = 2;
y_p1 = 6;
x_p2 = 0;
y_p2 = 6;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 6
fprintf("\nWALL 6\n")
x_p1 = 0;
y_p1 = 6;
x_p2 = 0;
y_p2 = 0;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 7
fprintf("\nWALL 7\n")
x_p1 = 3.5;
y_p1 = 1;
x_p2 = 4;
y_p2 = 1;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 8
fprintf("\nWALL 8\n")
x_p1 = 4;
y_p1 = 1;
x_p2 = 4;
y_p2 = 1.5;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 9
fprintf("\nWALL 9\n")
x_p1 = 4;
y_p1 = 1.5;
x_p2 = 3.5;
y_p2 = 1.5;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r
%% WALL 9
fprintf("\nWALL 10\n")
x_p1 = 3.5;
y_p1 = 1.5;
x_p2 = 3.5;
y_p2 = 1.0;
syms alpha r
sols =solve(r == x_p1*cos(alpha)+y_p1*sin(alpha),r == x_p2*cos(alpha)+y_p2*sin(alpha));
alpha = sols.alpha;
r = sols.r;
alpha
r