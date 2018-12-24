function R = generateRotationMatrix(a1,a2,a3,theta)

A1 = [0 -a3 a2; a3 0 -a1; -a2 a1 0];
R = eye(3)+sin(theta).*A1+(1-cos(theta)).*(A1*A1);