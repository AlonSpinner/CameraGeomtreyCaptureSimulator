function [camera,pairs,box,worldfig,worldAxes]=Test_CreateScene(worldAxes)
%note: any object not returned is deleted from function workspace
%activating its destructor!!

if nargin < 1
    worldfig=figure;
    worldAxes=axes(worldfig);
end
set(worldAxes,'DataAspectRatioMode','manual');
view(worldAxes,3);
hold(worldAxes,"on");
xlabel(worldAxes,'x'); ylabel(worldAxes,'y'); zlabel(worldAxes,'z'); 
%% Define Objects
%box
lengths=[3,1,1];
center=[0,0.5,0.5];
box=hyperrectangle3d(lengths,center,worldAxes);
box.plot('facecolor','none');
%Define Camera
position = [0,0.5,0.5]';
targetVector = [1,0,0]';
upVector = [0 0 1]';
camera=camera3d(worldAxes);
camera.computePose(position,targetVector,upVector)
camera.plot;

%-----------------Pair1
%plane1
P=[1.5  0.5  1
   1.5  1  0
   0.5 0 0.5];
plane1=plane3d(P,worldAxes);
%line1
P=[1.5,0.5,1;
    0.5,0,0.5];
line1=line3d(P,worldAxes);
%pairing
pairs{1}.plane=plane1;
pairs{1}.line=line1;

%-----------------Pair2
%plane2
P=[0.5  0  0
   1.5  0  0
   1.5  1  0
   0.5 1 0];
plane2=plane3d(P,worldAxes);
%line2
P=[1.5,1,0
    0.5,1,0];
line2=line3d(P,worldAxes);
%pairing
pairs{2}.plane=plane2;
pairs{2}.line=line2;

%% plot
plane1.plot('facecolor',[0.5,0.5,0.5]);
line1.plot('color',[1,0,0],'linewidth',2);
plane2.plot('facecolor',[0.5,0.8,0.5]);
line2.plot('color',[1,0,0],'linewidth',2);