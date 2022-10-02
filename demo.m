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
color = imread("000004.jpg");
depth = imread("000004.png");

% Json filename
json_filename = [];

% Output point cloud filename
file_name = 'output.pcd';

% Camera intrinsics
camera_intrinsic.cx              = 964.957;
camera_intrinsic.cy              = 522.586;
camera_intrinsic.fx              = 1390.53;
camera_intrinsic.fy              = 1386.99;
camera_intrinsic.depth_scale     = 1000;
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