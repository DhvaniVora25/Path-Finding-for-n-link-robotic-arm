p0 = [0; 0; 0];

fprintf('How many number of links do you want?');
n = input('');
fprintf('Enter the end position of each link:\n');
links = zeros(3,n);
for i=1:1:n
    fprintf('Link %d:',i);
    links(:,i) = input('');
end     

for i=1:1:n
    if i==1
        vectarrow(p0,links(:,i),'b');
        hold on;
    else
        vectarrow(links(:,i-1),links(:,i),'b');
        hold on;
    end
end    

transformations = zeros(4,4,n-1);

for i=2:1:n
    x = links(:,i-1);
    y = links(:,i);
    r = vrrotvec(x,y);
    R = generateRotationMatrix(r(1),r(2),r(3),r(4));
    t = y-R*x;
    se = [R,t;0,0,0,1];
    transformations(:,:,i-1) = se;
end    

fprintf('input the link number you want to rotate (the link at origin is the first link and the last one is %dth link)',n);
ln = input('');
fprintf('Enter the direction of rotation');
dir = input('');
fprintf('Enter the angle of rotation');
angle = input('');
t1 = zeros(3,1);
R = generateRotationMatrix(dir(1),dir(2),dir(3),angle);
SE = [R,t1;0,0,0,1];

last_se = eye(4);
new_se = SE;
link = links(:,ln);
    
if ln==1
    p0 = [0;0;0];
else
    p0 = links(:,ln-1);
end    

for i=ln:1:n
    
    if i~=ln
        last_se = last_se*transformations(:,:,i-1);
        new_se = new_se*transformations(:,:,i-1);
        geodesic_se(last_se,new_se,link,p0);
    else
        geodesic_se(last_se,new_se,link,p0);
    end
end    
% 
% a1 = zeros(3,1);
% for i = 1:1:3
%     a1(i) = input('');
% end    
% a1 = a1/norm(a1);
% fprintf('\nAngle : ');
% theta1 = input('');
% a = a1(1);
% b = a1(2);
% c = a1(3);
% 
% R1 = generateRotationMatrix(a,b,c,theta1);
% fprintf('\nEnter the translation:');
% t1 = zeros(3,1);
% for i=1:1:3
%     t1(i) = input('');
% end    
% SE1 = [R1,t1;0 0 0 1];
% 
% xr1 = R1*x_axis + t1;
% yr1 = R1*y_axis + t1;
% zr1 = R1*z_axis + t1;
% vectarrow(p0+t1,xr1,'r');
% hold on;
% vectarrow(p0+t1,yr1,'g');
% hold on;
% vectarrow(p0+t1,zr1,'b');
% hold on;
% 
% fprintf('Enter the ending direction and angles of rotation\n');
% fprintf('Direction : ');
% a2 = zeros(3,1);
% for i = 1:1:3
%     a2(i) = input('');
% end    
% fprintf('\nAngle : ');
% a2 = a2/norm(a2);
% theta2 = input('');
% 
% a = a2(1);
% b = a2(2);
% c = a2(3);
% 
% R2 = generateRotationMatrix(a,b,c,theta2);
% fprintf('\nEnter the translation:');
% t2 = zeros(3,1);
% for i=1:1:3
%     t2(i) = input('');
% end    
% SE2 = [R2,t2;0 0 0 1];
% 
% xr2 = R2*x_axis + t2;
% yr2 = R2*y_axis + t2;
% zr2 = R2*z_axis + t2;
% vectarrow(p0+t2,xr2,'r');
% hold on;
% vectarrow(p0+t2,yr2,'g');
% hold on;
% vectarrow(p0+t2,zr2,'b');
% hold on;
% %%%%% input taken
% 
% TR1 = logm(SE1);
% TR2Id = logm(inv(SE1)*SE2);
% TR2 = TR2Id;
% tempP1 = [0;0;0];
% 
% for i=0:0.1:1
%     R = expm(i.*TR2);
%     tempx1 = SE1*(R*[x_axis;1]);
%     vectarrow(p0+t1+(i*(t2-t1)),[tempx1(1,1);tempx1(2,1);tempx1(3,1)],'r');
%     hold on;
%     tempy1 = SE1*(R*[y_axis;1]);
%     vectarrow(p0+t1+(i*(t2-t1)),[tempy1(1,1);tempy1(2,1);tempy1(3,1)],'g');
%     hold on;
%     tempz1 = SE1*(R*[z_axis;1]);
%     vectarrow(p0+t1+(i*(t2-t1)),[tempz1(1,1);tempz1(2,1);tempz1(3,1)],'b');
%     hold on;
% end    