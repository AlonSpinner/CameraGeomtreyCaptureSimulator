%% This script create data for slam
%cameraP - camera poses
%cameraTargetVector - camera orientation
%Z(ii,jj,1:2) - [u,v] of point jj in pose ii


a = 3; % side length of world containing box
n = 12; % number of points
dt = 0.1;
T=10;
tPointsAmount = floor(T/dt);
STDv = 2;

[box,p3d,worldfig,worldAxes]=Slam_CreateScene(a,n);

%stupid matlab syntax. cell arrays can be converted into varargin
%https://www.mathworks.com/matlabcentral/answers/8266-convert-array-to-argument-list
p3dCell = cell(n,1);
for ii=1:n, p3dCell{ii}=p3d(ii); end

%% Camera Track
theta = linspace(0,2*pi,tPointsAmount);
x = a/2*cos(theta)';
y = a/2*sin(theta)';
cameraP = [x,y,zeros(tPointsAmount,1)];
cameraTargetVector = -cameraP/(a/2);
hold(worldAxes,'on');
h_track=plot3(worldAxes,cameraP(:,1),cameraP(:,2),cameraP(:,3),'color','black');
hold(worldAxes,'off');

%Define Camera
camera=camera3d(cameraP(1,:),cameraTargetVector(1,:),worldAxes);
camera.plot;


Z = zeros(tPointsAmount,n,2);
for ii=1:tPointsAmount
    camera.position=cameraP(ii,:);
    camera.computeProjMat(); %<--- dont forget. doesnt happen auto after position updated
    camera.targetVector=cameraTargetVector(ii,:);
    
    %gather data
    for jj=1:n
        [u,v] = camera.ProjectOnImage(p3d(jj));
        Z(ii,jj,:) = [u,v]+STDv*randn(1,2);
    end
    
    %plot image and camera
    camera.plot;
    camera.getframe(p3dCell{:}); %this here is exact, no noise added
    
    pause(0.01);
end

disp('data generated sucessfully, and is placed in workspace');