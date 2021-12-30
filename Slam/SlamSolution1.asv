%% Slam Solution 1
load('slamData.mat');

[nx,np,nz]=size(Z);
nd = 3; %x = [x,y,theta], l=[x,y,z]

Xbar = zeros(nx,1);

cols = nx*nd+np*nd; 
rows = (nx+np)*nz;
A = zeros(rows,cols);
for ii=1:nz:rows
    for jj=1:nd:cols
        A(ii:ii+nz-1,jj:jj+nd-1) = fh_x(xbar,lbar)
    end
end

