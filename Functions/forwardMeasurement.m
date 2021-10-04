function features=forwardMeasurement(camera,pair)
%rho- VL distance from image center
%theta- VL slope
%s- VP location on VL
%d- length of line segment in image
%p0 - center of line segment in image

%project Line
[u,v]=camera.ProjectOnImage(pair.line);
P=[u',v'];

d=sqrt(sum((P(1,:)-P(2,:)).^2));
p0=mean(P);

VL=camera.K'\pair.plane.n; %VL ~ [a,b,c] of line
rho=abs(VL(3))/sqrt((VL(1)^2+VL(2)^2+eps)); %proof in wiki https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
theta=-VL(1)/(VL(2)+eps); %ax+by+c=0 ->y=-(a/b)*x-(c/b).
    
s=0;    

features=[rho,theta,s,d,p0];
end