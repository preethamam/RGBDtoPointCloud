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
% Usage: obj                   = rgbd2pointcloud(color, depth, cam_intrinsics, json_filename)
%        
% Properties:
%           color              - Number of nodes in the graph
%           depth              - Radius that nodes within this radius will have edges connected
%           json_filename      - Optional positions for nodes in the graph
%           cam_intrinsics     - Dimension of the graph
%
% Outputs: 
%
%           G                  - Graph object for the random geometric graph
%           coor               - coordinates of all each point in the graph
%
%--------------------------------------------------------------------------
% Example 1: Generate a 2D geometric graph with 25 nodes using brutal force method
% node_num = 25;      
% radius = 0.5;  
% pos = [];
% dim = 2;       
% method = 'BruteForce';         
% showplot = 1;
% [G,coor] = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot);
%
% Example 2: Generate a 2D geometric graph with 25 nodes using KDTree method
% node_num = 25;      
% radius = 0.5;  
% pos = [];
% dim = 2;       
% method = 'KDTree';         
% showplot = 1;
% [G,coor] = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot);
%
% Example 3: Generate a 3D geometric graph with 25 nodes using KDTree method
% node_num = 25;      
% radius = 0.1;  
% pos = rand(100,3);
% dim = 3;       
% method = 'KDTree';         
% showplot = 1;
% [G,coor] = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot);


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


