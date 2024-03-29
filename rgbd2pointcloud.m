%%***********************************************************************%
%*                           RGBD to point cloud                        *%
%*          Generates point cloud from the color and depth image        *%
%*                                                                      *%
%*                                                                      *%
%* Author: Dr. Preetham Manjunatha                                      *%
%* Github link: https://github.com/preethamam                           *%
%* Date: 10/02/2022                                                     *%
%************************************************************************%
%
%************************************************************************%
%
% Usage: obj                   = rgbd2pointcloud(color, depth, cam_intrinsics, ...
%                                                json_filename)
%        
% Properties:
%           color              - Color image or file name
%           depth              - Depth image or file name
%           json_filename      - JSON file of camera intrinsic parameters
%           cam_intrinsics     - camera intrinsic parameters (if no JSON
%                                file is provided)
%
% Outputs: 
%
%           xyz                - X, Y and Z points
%           rgb                - coordinates of all each point in the graph
%           Point cloud file   - Output point cloud .ply or .pcd file
%
%--------------------------------------------------------------------------
% Example 1: Generate a 3D point cloud without JSON file
% Read the RGB and D images
% color = imread("redwood_1.png");
% depth = imread("redwood_1d.png");
% 
% Json filename
% json_filename = [];
% 
% Output point cloud filename
% file_name = 'output.pcd';
% 
% Camera intrinsics
% camera_intrinsic.cx              = 964.957;
% camera_intrinsic.cy              = 522.586;
% camera_intrinsic.fx              = 1390.53;
% camera_intrinsic.fy              = 1386.99;
% camera_intrinsic.depth_scale     = 1000; % Depth scale (constant) 
%                                    to convert mm to m vice-versa
% camera_intrinsic.width           = 640;
% camera_intrinsic.height          = 480;
%
% obj = rgbd2pointcloud(color, depth, camera_intrinsic, json_filename);
% [xyz, rgb] = obj.xyz_rgb();
% obj.write2file(xyz, rgb, file_name)
%
%--------------------------------------------------------------------------
%
% Example 2: Generate a 3D point cloud with JSON file
% Read the RGB and D images
% color = imread("redwood_1.png");
% depth = imread("redwood_1d.png");
% 
% Json filename
% json_filename = 'camera.json';
% 
% Output point cloud filename
% file_name = 'output.pcd';
%
% obj = rgbd2pointcloud(color, depth, camera_intrinsic, json_filename);
% [xyz, rgb] = obj.xyz_rgb();
% obj.write2file(xyz, rgb, file_name)

% Class definition
classdef rgbd2pointcloud < handle
    
    % Public properties
    properties
        color
        depth
        json_filename
        cam_intrinsics
    end

    % Public methods
    methods
        % Constructor
        function obj = rgbd2pointcloud(color, depth, cam_intrinsics, json_filename)
            if isstring(color)
                obj.color = imread(color);
            else
                obj.color = color;
            end

            if isstring(depth)
                obj.depth = double(imread(depth));
            else
                obj.depth = double(depth);
            end

            obj.json_filename = json_filename;
            obj.cam_intrinsics = cam_intrinsics;
        end

        % Get the XYZ and RGB data
        function [xyz, rgb] = xyz_rgb(obj)
            cam_params = get_camera_intrinsics(obj);
    
            xx = 1 : cam_params.width;
            yy = 1 : cam_params.height;
            [xv, yv] = meshgrid(xx, yy);
                  
            u = reshape(xv', 1, []);
            v = reshape(yv', 1, []);
    
            z = reshape(obj.depth', 1, []) / cam_params.depth_scale;
            x = (u - cam_params.cx) .* z / cam_params.fx;
            y = (v - cam_params.cy) .* z / cam_params.fy;
            
            xyz = [x; y; z]';
            rgb = reshape(pagetranspose(obj.color), [], 3);
        end 

        % Write to the point cloud
        function write2file(obj, xyz, rgb, file_name)
            ptCloud = pointCloud(xyz, Color = rgb);
            pcwrite(ptCloud,file_name, 'Encoding', 'compressed')
        end
    end

    % Private methods
    methods (Access = private)
        % JSON reader
        function cam_params = readJSON(obj)
            fid = fopen(obj.json_filename); 
            raw = fread(fid,inf); 
            str = char(raw'); 
            fclose(fid); 
            
            cam_params = jsondecode(str);
        end

        % Get the camera intrinsics
        function cam_params = get_camera_intrinsics(obj)
            if ~isempty(obj.json_filename)
                cam_params = readJSON;
            else
                cam_params = obj.cam_intrinsics;
            end
        end        
    end
end


