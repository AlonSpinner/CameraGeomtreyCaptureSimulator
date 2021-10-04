function R=EvalBezCrv_DeCasteljau(Q,q)
%{
Evaluates Bezier Curve by given nodes and parameter
value

Q - nodes in format [x] or [x,y,z] dimension matrix. top row is q=0.
q - running parameter of bezier curve. 0=<q<=1
R - [x] or [x,y,z] format. point on bezier curve
https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/de-casteljau.html
%}
Q0=Q;
n=size(Q0,1)-1; %degree of bezier polynomial
R=zeros(size(q,1),size(Q,2));
for jj=1:length(q)
    Q=Q0;
    for kk=1:(n+1)
        for ii=1:(n+1-kk) %doesnt enter in first iteration
            Q(ii,:)=(1-q(jj))*Q(ii,:)+q(jj)*Q(ii+1,:);
        end
    end
    R(jj,:)=Q(1,:);
end
end