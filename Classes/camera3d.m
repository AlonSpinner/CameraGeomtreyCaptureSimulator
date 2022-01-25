classdef camera3d < handle
    properties
        %Intrinsics
        h(1,1)=512;
        w(1,1)=512;
        fx(1,1)=512; %1 meter length in 1 meter disatnce will apear as fx pixels
        fy(1,1)=512; %1 meter length in 1 meter disatnce will apear as fx pixels
        px(1,1)=512/2;
        py(1,1)=512/2;
        K(3,3) =[512,0,512/2; 0,512,512/2; 0,0,1];
        %Extrinsics
        pose(4,4) = eye(4); %t(G)G->C, RG2C
        %Intrinsics+Extrinsics
        ProjMat(3,4)
        %Physical
        cameraSize(1,1)=0.1;
        %Graphics
        worldAxes(1,1)=gobjects(1,1);
        imagePlaneFig(1,1)=gobjects(1,1);
        imagePlaneAxes(1,1)=gobjects(1,1);
        graphicHandle(1,1)=gobjects(1,1);
    end
    methods
        function obj=camera3d(worldAxes)
            %all vec inputs are column vecs, see method 'computePose'
            if nargin<1 %worldAxes not provided
                worldAxes=gca;
            end
            obj.worldAxes=worldAxes;
        end
        function pose = computePose(obj,pos,targetVec,upVec)
            if nargin == 2 %full pose was given in pos - a 4x4 matrix
                obj.pose = pos; 
                obj.computeProjMat(); %if updated pose, also need to update projmat
                return
            end
            
            %ELSE
            %all inputs are column vecs
                % pos - translation vector of camera pin hole center in
                % global frame. %t(G)G->C
                % targetVec and upVec are for rotation

            %follows Matlab Camrea Conventions: https://www.mathworks.com/help/vision/gs/coordinate-systems.html
            % Z is forward #targetVec
            % Y is pointing down -#upVec
            % X is to the right
            
            x = cross(targetVec,upVec);
            RGtC=[x,-upVec,targetVec]; 
            
            pose=[RGtC,pos;... 
                [0 0 0 1]];
            obj.pose = pose; %t(G)G->C, RG2C
            obj.computeProjMat(); %if updated pose, also need to update projmat
        end
        function plot(obj)
            plotpose=rigid3d(obj.pose');
            
            if isvalid(obj.graphicHandle) &&...
                    isa(obj.graphicHandle,'vision.graphics.Camera') %only update
                obj.graphicHandle.AbsolutePose=plotpose;
            else
                hold(obj.worldAxes,'on');
                obj.graphicHandle=plotCamera(...
                    'Parent',obj.worldAxes,...
                    'AbsolutePose',plotpose,...
                    'size',obj.cameraSize);
                hold(obj.worldAxes,'off');
            end
        end
        function image=getframe(obj,varargin)
            if isvalid(obj.imagePlaneFig) &&... %if imagePlane existed, clear it
                    isa(obj.imagePlaneFig,'matlab.ui.Figure')
                cla(obj.imagePlaneAxes);
            else %else.. create a new window
                obj.imagePlaneFig=figure('color',[1,1,1],'Visible','off'); %open new figure
                obj.imagePlaneAxes=axes('parent',obj.imagePlaneFig,...
                    'view',[0 90],...
                    'XLim',[0,obj.w],...
                    'YLim',[0,obj.h]);
                axis(obj.imagePlaneAxes,'manual'); %mode - manual, style - image or not?
            end
            
            cla(obj.imagePlaneAxes);
            hold(obj.imagePlaneAxes,'on');
            for ii=1:length(varargin)
                [u,v]=ProjectOnImage(obj,varargin{ii});
                switch class(varargin{ii})
                    case 'plane3d'
                        patch(obj.imagePlaneAxes,'XData',u,'YData',v,...
                            'FaceColor',varargin{ii}.graphicHandle.FaceColor,...
                            'EdgeColor',varargin{ii}.graphicHandle.EdgeColor,...
                            'FaceAlpha',varargin{ii}.graphicHandle.FaceAlpha);
                    case 'line3d'
                        plot(obj.imagePlaneAxes,u,v,...
                            'Color',varargin{ii}.graphicHandle.Color,...
                            'LineWidth',varargin{ii}.graphicHandle.LineWidth,...
                            'LineStyle',varargin{ii}.graphicHandle.LineStyle);
                    case 'point3d'
                        scatter(obj.imagePlaneAxes,u,v,...
                            'Marker',varargin{ii}.graphicHandle.Marker,...
                            'MarkerFaceColor',varargin{ii}.graphicHandle.MarkerFaceColor);
                end
            end
            
            F = getframe(obj.imagePlaneAxes);% Grab the rendered frame
            image=F.cdata;
        end
        function [u,v]=ProjectOnImage(obj,geo3d)
            P=geo3d.P;
            m=size(P,1); %number of points
            X=[P'; %transpose here so X is [x;y;z;1]
                ones(1,m)];
            x=obj.ProjMat*X;
            x=(x./(x(3,:)+eps));
            u=x(1,:);
            v=x(2,:);
            v=obj.h-v; %flip vertifcal axis of camera for image
        end
        function computeProjMat(obj)
            Rwtc=obj.pose(1:3,1:3)';
            O=obj.pose(1:3,4);
            obj.ProjMat=obj.K*[Rwtc,-Rwtc*O];
        end
        function [fhz,fhz_x,fhz_l] = compute2DMeasurementModel(obj)
            %landmarks - [x,y,z]'
            %pose - [x,y,theta]'
            x = sym('x',[3,1]);
            l = sym('l',[3,1]);
            R = [cos(x(3)) -sin(x(3)) 0;
                sin(x(3)) cos(x(3))  0;
                0 0 1];
            t = [x(1); x(2); 0];
            P = obj.K*[R,t];
            u_bar = P*[l;1];
            u = u_bar(1)/u_bar(3);
            v = u_bar(2)/u_bar(3);
            
            hz = [u;v];
            hz_x = jacobian(hz,x);
            hz_l = jacobian(hz,l);

            fhz = matlabFunction(hz,'vars',{x,l});
            fhz_x = matlabFunction(hz_x,'vars',{x,l});
            fhz_l = matlabFunction(hz_l,'vars',{x,l});
        end
        function delete(obj) %destructor
            delete(obj.graphicHandle);
        end
    end
end
%% Supporting Functions
function computeK(obj)
obj.K=[obj.fx,0,obj.px;
    0,obj.fy,obj.py;
    0,0,1];
end