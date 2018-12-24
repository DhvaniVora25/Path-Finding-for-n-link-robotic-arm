function in = inv_se(SE)
R = SE(1:3,1:3);
t = SE(1:3,4);
in = [inv(R),t;0,0,0,1];