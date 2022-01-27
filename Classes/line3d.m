classdef line3d < handle
    properties
        P(2,3);
        direction(3,1);
        length(1,1);
        
        worldAxes(1,1)=gobjects(1,1);
        graphicHandle(1,1)=gobjects(1,1);
    end
    methods
        function obj=line3d(P,worldAxes) %constructor
            if nargin<2 %worldAxes not provided
                worldAxes=gca;
            end
            P0=P(1,:); P1=P(2,:);
            D=P1-P0;
            L=vecnorm(D,2);
            t=D/L;
            
            obj.P=P;
            obj.direction=t;
            obj.length=L;
            obj.worldAxes=worldAxes;
        end
        function delete(obj) %destructor
            delete(obj.graphicHandle);
        end
        function plot(obj,varargin)
            x=obj.P(:,1); y=obj.P(:,2); z=obj.P(:,3);
            if isvalid(obj.graphicHandle) &&...
                    isa(obj.graphicHandle,'matlab.graphics.Line') %only update
                set(obj.graphicHandle,...
                    'XData',x,...
                    'YData',y,...
                    'ZData',z,...
                    varargin{:});
            else
                hold(obj.worldAxes,'on');
                obj.graphicHandle=plot3(x,y,z,...
                    'Parent',obj.worldAxes,...
                    varargin{:});
                hold(obj.worldAxes,'off');
            end
            
        end
    end
end