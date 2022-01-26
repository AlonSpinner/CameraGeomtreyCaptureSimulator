
%% This script creates data for our slam problem

%-----------------Hyper Parameters:
a = 3; % side length of world containing box
nLM = 12; % number of landmarks
dt = 0.1; %[s]
T=10; %[s]
tPointsAmount = floor(T/dt); %amount of frames

%----------------Measurement Noise;
STDv = 2; %visual
STDt = a/10; %translation
%no rotation noise implemented
%%
fig = figure('color',[1,1,1],'units','normalized','Position',[0.2,0.2,0.6,0.4]);
worldAxes = subplot(1,2,1,'parent',fig);
cameraAxes = subplot(1,2,2,'parent',fig);
%% build the Scene
[box,p3d]=SLAM_CreateScene(a,nLM,worldAxes);
%stupid matlab syntax. cell arrays can be converted into varargin
%https://www.mathworks.com/matlabcentral/answers/8266-convert-array-to-argument-list
p3dCell = cell(nLM,1);
for ii=1:nLM, p3dCell{ii}=p3d(ii); end %need this data structure for camera's getframe method.
XYZ = cell2mat(arrayfun(@(p) p.P,p3d,'UniformOutput',false)');
Color = cell2mat(arrayfun(@(p) p.graphicHandle.CData,p3d,'UniformOutput',false)');
lmTable = table(XYZ,Color);
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
    tCinG = traj.pos(ii,:)';
    tVec = traj.TargetVector(ii,:)';
    upVec = traj.upVector(ii,:)';
    xVec = cross(tVec,upVec);
    RC2G=[xVec,-upVec,tVec];
    traj.poses(:,:,ii)=[RC2G,tCinG;...
        [0 0 0 1]];
end
traj.frames = cell(1,tPointsAmount); %initalize for frames, that will be collected later

hold(worldAxes,'on');
h_track=plot3(worldAxes,traj.pos(:,1),traj.pos(:,2),traj.pos(:,3),'color','black');
hold(worldAxes,'off');
%Define Camera
camera=camera3d(worldAxes);
camera.plot;

%% Simulate
Z = zeros(tPointsAmount,nLM,2);
O = zeros(tPointsAmount-1,4,4);
for ii=1:tPointsAmount
    camera.computePose(traj.poses(:,:,ii));
    
    %gather visual data, assuming perfect data assosication, and always
    %visible landmarks
    for jj=1:nLM
        [u,v] = camera.ProjectOnImage(p3d(jj));
        Z(ii,jj,:) = [u,v]+STDv*randn(1,2);
    end

    if ii > 1
        %reminder,traj.pose holds info as Rii  = Rii2G ; tii = t(G)G->ii
        Tii = traj.poses(:,:,ii); Rii = Tii(1:3,1:3); tii = Tii(1:3,4);
        Tiim1 = traj.poses(:,:,ii-1); Riim1 = Tiim1(1:3,1:3); tiim1 = Tiim1(1:3,4);
        
        R = Rii'*Riim1;  
        t = Rii'*(tiim1-tii)+STDt*randn(3,1); %t(ii)_ii->iim1
        O(ii-1,:,:) = [R,t;0,0,0,1];
    end
    %plot image and camera
    camera.plot;
    frame = camera.getframe(p3dCell{:}); %this here is exact, no noise added
    traj.frames{ii} = frame;
    image(cameraAxes,frame);
end

cameraInstrincs.K = camera.K;
cameraInstrincs.imageSize = [camera.w,camera.h];

disp('data generated sucessfully, and is placed into workspace');
%%  Save Data to file
proj = matlab.project.rootProject;
filename = fullfile(proj.RootFolder,'SLAM','SLAMData');
save(filename,'traj','lmTable','cameraInstrincs','Z','O','STDv','STDt');

disp('Now storing it in /SLAM/SLAMData.mat')