%% Load Data and Import Solver
data = load('SLAMData.mat');
%% Extrac Physical
fx = data.cameraInstrincs.K(1,1);
fy = data.cameraInstrincs.K(2,2);
px = data.cameraInstrincs.K(1,3);
py = data.cameraInstrincs.K(2,3);
imageSize = data.cameraInstrincs.imageSize;
focalLength = [fx,fy];
principalPoint = [px,py];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
%% Extract measurements
O = data.O;
Z = data.Z;
tAmount = size(Z,1);
lmAmount = size(Z,2);
STDv = data.STDv;
%% Extract ground truth
trajgt = data.traj;
T1tG =  trajgt.poses(:,:,1); %use this just for laying out on same axes
%% Integrate odometrey for initial estimate
initialEstimate = gtsam.Values;
T = eye(4);
initialEstimate.insert(gtsam.symbol('x',1),gtsam.Pose3(T1tG*T));
for ii=1:size(O,1)
    Trel = inv(squeeze(O(ii,:,:)));
    T = Trel*T;
    initialEstimate.insert(gtsam.symbol('x',ii+1),gtsam.Pose3(T1tG*T));
end
figure();
gtsam.plot3DTrajectory(initialEstimate, '*-k' ,200);
view(3);
grid('on'); view(3); axis('equal');
xlabel('x'); ylabel('y'); zlabel('z');
%% Solve SLAM based VAN
%following the example in MonocularVOExample
Sr = 0 * ones(3,1);
St = 0.1 * ones(3,1);
noiseModel = gtsam.noiseModel.Diagonal.Sigmas([Sr ; St ]);

graph = gtsam.NonlinearFactorGraph;
for ii=2:tAmount
    pii = squeeze(Z(ii,:,:));
    piim1 = squeeze(Z(ii-1,:,:));
    [E,~,status] = estimateEssentialMatrix(piim1,pii,intrinsics);
    [R,t] = relativeCameraPose(E,intrinsics,pii,piim1);
    T = [R,t';0,0,0,1];
    graph.add(gtsam.BetweenFactorPose3(gtsam.symbol('x',ii),...
        gtsam.symbol('x',ii-1),...
        gtsam.Pose3(T),...
        noiseModel));
end
%%
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
% marginals = gtsam.Marginals(graph, result);
%%
figure();
gtsam.plot3DTrajectory(result, '*-b') ,[],[],marginals)
view([0,-1,0]); axis equal; axis tight; grid on;
title('\color{blue}Optimized Trajectory');