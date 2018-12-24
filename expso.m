function [theta,y] = expso(A)
%A1 = A;
a = A(3,2);
b = A(1,3);
c = A(2,1);
theta = sqrt(a*a + b*b + c*c);
%B1 = [a*a a*b a*c; a*b b*b b*c; a*c b*c c*c];

%y = eye(3)+sin(theta).*A+(1-cos(theta)).*(A*A);
y = expm(A);

end