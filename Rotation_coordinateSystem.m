clear all;
close all;
p0 = [0; 0; 0];
x_axis = [1; 0; 0];
y_axis = [0; 1; 0];
z_axis = [0; 0; 1];

fprintf('Enter the start direction and angles of rotation\n');
fprintf('Direction : ');
a1 = zeros(3,1);
for i = 1:1:3
    a1(i) = input('');
end    
a1 = a1/norm(a1);
fprintf('\nAngle : ');
theta1 = input('');
a = a1(1);
b = a1(2);
c = a1(3);

R1 = generateRotationMatrix(a,b,c,theta1);

xr1 = R1*x_axis;
yr1 = R1*y_axis;
zr1 = R1*z_axis;
vectarrow(p0,xr1,'r');
hold on;
vectarrow(p0,yr1,'g');
hold on;
vectarrow(p0,zr1,'b');
hold on;

fprintf('Enter the ending direction and angles of rotation\n');
fprintf('Direction : ');
a2 = zeros(3,1);
for i = 1:1:3
    a2(i) = input('');
end    
a2 = a2/norm(a2);
fprintf('\nAngle : ');
theta2 = input('');

a = a2(1);
b = a2(2);
c = a2(3);

R2 = generateRotationMatrix(a,b,c,theta2);

xr2 = R2*x_axis;
yr2 = R2*y_axis;
zr2 = R2*z_axis;
vectarrow(p0,xr2,'r');
hold on;
vectarrow(p0,yr2,'g');
hold on;
vectarrow(p0,zr2,'b');
hold on;

%%%%%input taken

%[thetaTR1,TR1] = logSO(R1);
[thetaTR2Id, TR2Id] = logSO(R1'*R2);
TR2 = TR2Id;
tempP1 = [0;0;0];

for i=0:0.1:1
    [theta,S] = expso(i.*TR2);
    tempx1 = R1*S*x_axis;
    tempy1 = R1*S*y_axis;
    tempz1 = R1*S*z_axis;
    vectarrow(p0,tempx1,'r');
    hold on;
    vectarrow(p0,tempy1,'g');
    hold on;
    vectarrow(p0,tempz1,'b');
    hold on;
    
end    