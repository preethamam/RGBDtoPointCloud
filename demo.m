%//%************************************************************************%
%//%*                        RGBD to point cloud                           *%
%//%*                                                                      *%
%//%*             Author: Dr. Preetham Manjunatha                          *%
%//%*             GitHub: https://github.com/preethamam                    *%
%//%*                                                                      *%
%//%************************************************************************%
%//%*                                                                      *%                             
%//%*             University of Southern california,                       *%
%//%*             Los Angeles, California.                                 *%
%//%************************************************************************%

%% Start parameters
%--------------------------------------------------------------------------
clear; close all; clc;
Start = tic;

%% Inputs
%--------------------------------------------------------------------------
% Read the RGB and D images
color = imread("redwood_847.png");
depth = imread("redwood_847d.png");

% Json filename
json_filename = [];

% Output point cloud filename
file_name = 'output.pcd';

% Camera intrinsics
camera_intrinsic.cx              = 319.5;
camera_intrinsic.cy              = 239.5;
camera_intrinsic.fx              = 525;
camera_intrinsic.fy              = 525;
camera_intrinsic.depth_scale     = 1000; % Depth scale (constant) to convert mm to m vice-versa
camera_intrinsic.width           = 640;
camera_intrinsic.height          = 480;

%% Object callback
%--------------------------------------------------------------------------
obj = rgbd2pointcloud(color, depth, camera_intrinsic, json_filename);
[xyz, rgb] = obj.xyz_rgb();
obj.write2file(xyz, rgb, file_name)

%% Display the point cloud
%--------------------------------------------------------------------------
figure;
pcshow('output.pcd', 'VerticalAxis', 'x', 'VerticalAxisDir', 'down')
xlabel("X")
ylabel("Y")
zlabel("Z")

%% End parameters
%--------------------------------------------------------------------------
Runtime = toc(Start);
disp(Runtime);