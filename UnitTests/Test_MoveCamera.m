%%
[camera,pairs,box,worldfig,worldAxes]=Test_CreateScene();
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

target=[1,0.5,0.5]';
for ii=1:length(p)
    camera.position=p(ii,:);
    camera.computeProjMat(); %should happen auto from camera.position
    D=target-camera.position;
    camera.targetVector=D/vecnorm(D,2);
    camera.plot;
    drawnow;
    pause(0.01);
    
    img=camera.getframe(pairs{1}.plane,...
        pairs{1}.line,...
        pairs{2}.plane,...
        pairs{2}.line);
end
delete(h_track);