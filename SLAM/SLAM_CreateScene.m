function [box,p3d,worldAxes]=SLAM_CreateScene(a,n,worldAxes)
if nargin < 3
    worldfig=figure;
    worldAxes=axes(worldfig);
end
%n = number of points
%a = side length of world containing box
%
% note: any object not returned is deleted from function workspace
%activating its destructor!!

s = 0.2; % Numeric factor here to scale the points togther
%% Define World Axes
set(worldAxes,'DataAspectRatioMode','manual');
view(worldAxes,3);
xlabel(worldAxes,'x'); ylabel(worldAxes,'y'); zlabel(worldAxes,'z');
%% Define Objects
%world containing box
lengths=[a,a,a];
center=[0,0,0];
box=hyperrectangle3d(lengths,center,worldAxes);
box.plot('facecolor','none');

%points
rng(0); %repeatability
P = s*a*(rand(n,3)-1/2); 
p3d = point3d.empty(n,0);
for ii=1:n
    p3d(ii) = point3d(P(ii,:),worldAxes);
end
%% plot
for ii=1:n
    p3d(ii).plot('filled');
end
end