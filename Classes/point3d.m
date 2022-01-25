classdef point3d < handle
    properties
        P(1,3);
        
        worldAxes(1,1)=gobjects(1,1);
        graphicHandle(1,1)=gobjects(1,1);
    end
    methods
        function obj=point3d(P,worldAxes) %constructor
            if nargin<2 %worldAxes not provided
                worldAxes=gca;
            end
            obj.P=P;
            obj.worldAxes=worldAxes;
        end
        function delete(obj) %destructor
            delete(obj.graphicHandle);
        end
        function plot(obj,varargin)
            x=obj.P(1); y=obj.P(2); z=obj.P(3);
            if isvalid(obj.graphicHandle) &&...
                    isa(obj.graphicHandle,'matlab.graphics.chart.primitive.Scatter') %only update
                set(obj.graphicHandle,...
                    'XData',x,...
                    'YData',y,...
                    'ZData',z,...
                    varargin{:});
            else
                hold(obj.worldAxes,'on');
                obj.graphicHandle=scatter3(x,y,z,...
                    'Parent',obj.worldAxes,...
                    varargin{:});
                hold(obj.worldAxes,'off');
            end
            
        end
    end
end