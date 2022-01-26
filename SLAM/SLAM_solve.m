%% Load Data and Import Solver
data = load('SLAMData.mat');
%Calibration matrix
fx = data.cameraInstrincs.K(1,1);
fy = data.cameraInstrincs.K(2,2);
px = data.cameraInstrincs.K(1,3);
py = data.cameraInstrincs.K(2,3);
K = gtsam.Cal3_S2([fx,fy,px,py]');

%Measurements
O = data.O;
Z = data.Z;
tAmount = size(Z,1);
lmAmount = size(Z,2);
STDv = data.STDv;

%Extract ground truth
gt_traj = data.traj;
gt_T1tG =  gt_traj.poses(:,:,1);
gt_lmTable = data.lmTable;
gt_lm1 = data.lmTable.XYZ(1,:);
%% Optimizer Parameters
pointNoiseSigma = 0.01*0.1;
poseNoiseSigmas = 0.001*[0.001 0.001 0.001 0.1 0.1 0.1]';
%% Integrate odometrey for initial estimate
initialEstimate = gtsam.Values;
T = eye(4);
initialEstimate.insert(gtsam.symbol('x',1),gtsam.Pose3(gt_T1tG*T));
for ii=1:size(O,1)
    Trel = inv(squeeze(O(ii,:,:)));
    T = Trel*T;
    initialEstimate.insert(gtsam.symbol('x',ii+1),gtsam.Pose3(gt_T1tG*T));
end

for ii=1:lmAmount
    p =  gtsam.Point3(data.lmTable.XYZ(ii,:)');
    p = p.retract(0.1*randn(3,1)); %add noise
    initialEstimate.insert(gtsam.symbol('p',ii), p);
end

figure();
gtsam.plot3DTrajectory(initialEstimate, '*-k' ,200);
view(3); hold('on');
gtsam.plot3DPoints(initialEstimate,'m');
grid('on'); view(3); axis('equal');
xlabel('x'); ylabel('y'); zlabel('z');
title('Initial Conditions');
%% add factors for all Z measurements
%following the example: SFMExample.m
graph = gtsam.NonlinearFactorGraph;
measurementNoise = gtsam.noiseModel.Isotropic.Sigma(2,2.0);
for ii=1:size(Z,1) %camera poses
    for jj=1:size(Z,2) %landmarks
        uv = squeeze(Z(ii,jj,:));
        graph.add(gtsam.GenericProjectionFactorCal3_S2(gtsam.Point2(uv),...
            measurementNoise, gtsam.symbol('x',ii), gtsam.symbol('p',jj),K));
    end
end
%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = gtsam.noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
graph.add(gtsam.PriorFactorPose3(gtsam.symbol('x',1), gtsam.Pose3(gt_T1tG), posePriorNoise));
pointPriorNoise  = gtsam.noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
graph.add(gtsam.PriorFactorPoint3(gtsam.symbol('p',1), gtsam.Point3(gt_lm1'), pointPriorNoise));
%% Fine grain optimization, allowing user to iterate step by step
parameters = gtsam.LevenbergMarquardtParams;
parameters.setlambdaInitial(1.0);
parameters.setVerbosityLM('trylambda');

optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);

for i=1:5
    optimizer.iterate();
end
result = optimizer.values();
result.print(sprintf('\nFinal result:\n  '));
%% Plot results with covariance ellipses
marginals = gtsam.Marginals(graph, result);

figure;
hold on;

gtsam.plot3DPoints(result, [], marginals);
% gtsam.plot3DPoints(result);
gtsam.plot3DTrajectory(result,'*',1, 1,marginals);
% gtsam.plot3DTrajectory(result, '*', 1, 8, marginals);

axis equal; grid on;
view(3)
colormap('hot')
xlabel('x'); ylabel('y'); zlabel('z');
title('results');