p0 = [0; 0; 0];
p1 = [1; 1; 1];

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
fprintf('\nEnter the translation:');
t1 = zeros(3,1);
for i=1:1:3
    t1(i) = input('');
end    
SE1 = [R1,t1;0 0 0 1];

pR1 = R1*p1 + t1;
vectarrow(p0+t1,pR1);
hold on;

fprintf('Enter the ending direction and angles of rotation\n');
fprintf('Direction : ');
a2 = zeros(3,1);
for i = 1:1:3
    a2(i) = input('');
end    
fprintf('\nAngle : ');
a2 = a2/norm(a2);
theta2 = input('');

a = a2(1);
b = a2(2);
c = a2(3);

R2 = generateRotationMatrix(a,b,c,theta2);
fprintf('\nEnter the translation:');
t2 = zeros(3,1);
for i=1:1:3
    t2(i) = input('');
end    
SE2 = [R2,t2;0 0 0 1];

pR2 = R2*p1 + t2;
vectarrow(p0+t2,pR2);
hold on;

%%%%% input taken

TR1 = logm(SE1);
TR2Id = logm(inv(SE1)*SE2);
TR2 = TR2Id;
tempP1 = [0;0;0];

for i=0:0.1:1
    R = expm(i.*TR2);
    tempP1 = SE1*(R*[p1;1]);
    vectarrow(p0+t1+(i*(t2-t1)),[tempP1(1,1);tempP1(2,1);tempP1(3,1)]);
    hold on;
end    