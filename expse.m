function [translate, R] = expse(se)
so = se([1 2 3],[1 2 3]);
[theta,R] = expso(so);
a = so(3,2);
b = so(1,3);
c = so(2,1);
theta = sqrt(a*a + b*b + c*c);
%U = se([1 2 3],4);
%V = eye(3) + ((1-cos(theta))/(theta*theta)).*so + ((theta-sin(theta))/(theta*theta*theta)).*(so*so);
%translate = V*U;
translate = expm(se);