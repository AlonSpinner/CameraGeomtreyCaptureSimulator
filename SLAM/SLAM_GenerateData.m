
%% This script creates data for our slam problem

% Hyper Parameters:
a = 3; % side length of world containing box
n = 12; % number of points
dt = 0.1;
T=10;
tPointsAmount = floor(T/dt);
STDv = 2;
%%
fig = figure('color',[1,1,1],'units','normalized','Position',[0.2,0.2,0.6,0.4]);
worldAxes = subplot(1,2,1,'parent',fig);
cameraAxes = subplot(1,2,2,'parent',fig);
%% build the Scene
[box,p3d]=SLAM_CreateScene(a,n,worldAxes);
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
traj.upVector = repmat([0,0,1],[tPointsAmount,1]);
traj.poses = zeros(4,4,tPointsAmount);
for ii=1:tPointsAmount
    pos = traj.pos(ii,:)';
    tVec = traj.TargetVector(ii,:)';
    upVec = traj.upVector(ii,:)';
    x = cross(tVec,upVec);
    RGtC=[x,-upVec,tVec];
    traj.poses(:,:,ii)=[RGtC,pos;...
        [0 0 0 1]];
end

hold(worldAxes,'on');
h_track=plot3(worldAxes,traj.pos(:,1),traj.pos(:,2),traj.pos(:,3),'color','black');
hold(worldAxes,'off');
%Define Camera
camera=camera3d(worldAxes);
camera.plot;

%% Simulate
Z = zeros(tPointsAmount,n,2);
for ii=1:tPointsAmount
    camera.computePose(traj.poses(:,:,ii));
    
    %gather data
    for jj=1:n
        [u,v] = camera.ProjectOnImage(p3d(jj));
        Z(ii,jj,:) = [u,v]+STDv*randn(1,2);
    end
    %plot image and camera
    camera.plot;
    frame = camera.getframe(p3dCell{:}); %this here is exact, no noise added
    image(cameraAxes,frame);
    
    pause(0.01);
end
%% 
disp('data generated sucessfully, and is placed into workspace');
disp('Now storing it in /BundleAdjustment/BAData.mat')
disp('Reminder: function handles accept x - [x,y,theta] and then l - [lx,ly,lz]');

[fhz,fhz_x,fhz_l] = camera.compute2DMeasurementModel;


proj = matlab.project.rootProject;
filename = fullfile(proj.RootFolder,'SLAM','SLAMData');
save(filename,'traj','fhz','fhz_x','fhz_l','Z','STDv');