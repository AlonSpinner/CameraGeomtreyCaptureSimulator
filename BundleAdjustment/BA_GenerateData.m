
%% This script creates data for our slam problem

% Hyper Parameters:
a = 3; % side length of world containing box
n = 12; % number of points
dt = 0.1;
T=10;
tPointsAmount = floor(T/dt);
STDv = 2;
%% build the Scene
[box,p3d,worldfig,worldAxes]=BA_CreateScene(a,n);
%stupid matlab syntax. cell arrays can be converted into varargin
%https://www.mathworks.com/matlabcentral/answers/8266-convert-array-to-argument-list
p3dCell = cell(n,1);
for ii=1:n, p3dCell{ii}=p3d(ii); end
%% Run Simulation for Data
theta = linspace(0,2*pi,tPointsAmount);
x = a/2*cos(theta)';
y = a/2*sin(theta)';
%% Build Trajectory
traj.pos = [x,y,zeros(tPointsAmount,1)];
traj.TargetVector = -traj.pos/(a/2);

hold(worldAxes,'on');
h_track=plot3(worldAxes,traj.pos(:,1),traj.pos(:,2),traj.pos(:,3),'color','black');
hold(worldAxes,'off');
%Define Camera
camera=camera3d(traj.pos(1,:),traj.TargetVector(1,:),worldAxes);
camera.plot;

%% Simulate
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
%% 
disp('data generated sucessfully, and is placed into workspace');
disp('Now storing it in /BundleAdjustment/BAData.mat')
disp('Reminder: function handles accept x - [x,y,theta] and then l - [lx,ly,lz]');

[fhz,fhz_x,fhz_l] = camera.compute2DMeasurementModel;
gt_pose = [cameraP(:,[1,2]),mod(theta+pi,2*pi)']; %angle to location + 180 to turn towards center

proj = matlab.project.rootProject;
filename = fullfile(proj.RootFolder,'BundleAdjustment','BAData');
save(filename,'gt_pose','fhz','fhz_x','fhz_l','Z','STDv');
