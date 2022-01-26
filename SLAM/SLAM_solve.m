%% Load Data and Import Solver
data = load('SLAMData.mat');
%% Extract measurements
O = data.O;
Z = data.Z;
tAmount = size(Z,1);
lmAmount = size(Z,2);
STDv = data.STDv;
%%
fig = figure;
ax = axes(fig); hold(ax,'on'); view(3)
T = eye(4);
for ii=1:size(O,1)
    Trel = inv(squeeze(O(ii,:,:)));
    T = Trel*T;
    scatter3(T(1,4),T(2,4),T(3,4),'filled');
    drawnow;
end
%% Integrate odometrey for initial estimate
initialEstimate = gtsam.Values;
T = eye(4);
initialEstimate.insert(gtsam.symbol('x',1),gtsam.Pose3(T));
for ii=2:size(O,1)
    Trel = inv(squeeze(O(ii,:,:)));
    T = Trel*T;
    initialEstimate.insert(gtsam.symbol('x',ii),gtsam.Pose3(T));
end
figure();
gtsam.plot3DTrajectory(initialEstimate, '*-k' ,200);
view(3); axis('equal');
%% Solve SLAM based VAN
%following the example in MonocularVOExample

graph = gtsam.NonlinearFactorGraph;
zModel = gtsam.noiseModel.Isotropic.Sigma(1,STDv); %Noise sigma is 1cm, assuming metric measurements

for ii=2:tAmount
    key = ii;
    for jj = 1:lmAmount %all lm visible the entire time
        pA = gtsam.Point2(squeeze(Z(ii,jj,:)));
        pB = gtsam.Point2(squeeze(Z(ii-1,jj,:)));
        graph.add(gtsam.EssentialMatrixFactor(key, pA, pB, zModel));
    end
end

initialEstimate = gtsam.Values;
%%
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n'));