%%
fig = figure('color',[1,1,1],'units','normalized','Position',[0.2,0.2,0.6,0.4]);
worldAxes = subplot(1,2,1,'parent',fig);
cameraAxes = subplot(1,2,2,'parent',fig);
[camera,pairs,box]=Test_CreateScene(worldAxes);
%% Take Camera for a stroll
Q=[-1,0,0]+[0,0.5,0.5
    0.6,0,0.6;
    0.6,0.6,0.6;
    0,0.5,0.5];
q=linspace(0,1,100);
p=EvalBezCrv_DeCasteljau(Q,q);

hold(worldAxes,'on');
h_track=plot3(worldAxes,p(:,1),p(:,2),p(:,3));
hold(worldAxes,'off');

targetPos=[1,0.5,0.5]';
upVec = [0,0,1]';
for ii=1:length(p)
    D = targetPos - p(ii,:)';
    targetVec = D/vecnorm(D);
    upVecAdjusted = cross(cross(targetVec,upVec),targetVec);
    upVecAdjusted = upVecAdjusted/vecnorm(upVecAdjusted);
    camera.computePose(p(ii,:)',targetVec,upVecAdjusted);
    camera.plot;
    
    frame=camera.getframe(pairs{1}.plane,...
        pairs{1}.line,...
        pairs{2}.plane,...
        pairs{2}.line);
    image(cameraAxes,frame);

    drawnow;
    pause(0.01);
end
delete(h_track);