l1 = 4;
l2 = 2;
link1 = [l1;0;0];
link2 = [l2;0;0];

fprintf('For link 1\n');
fprintf('Axis of rotation:');
a1 = input('');
fprintf('\nangle');
th1 = input('');
a1 = a1/norm(a1);

R1 = generateRotationMatrix(a1(1),a1(2),a1(3),th1);

fprintf('For link 2\n');
fprintf('Axis of rotation:');
a2 = input('');
fprintf('\nangle');
th2 = input('');
a2 = a2/norm(a2);

R2 = generateRotationMatrix(a2(1),a2(2),a2(3),th2);
t2 = R1*link1;
SE2 = [R2,t2;0,0,0,1];

link1_init = R1*link1;
link2_init1 = SE2*[link2;1];
link2_init = [link2_init1(1);link2_init1(2);link2_init1(3)];

vectarrow([0;0;0],link1_init,'g');
hold on;
vectarrow(link1_init, link2_init,'g');
hold on;

fprintf('Enter the final position of the end effector:');
pos = input('');

vectarrow([0;0;0],pos,'b');
hold on;

l1I = link1_init;
l2I = link2_init-link1_init;

a = norm(l1I);
b = norm(l2I);
c = norm(pos);

For the circle in 3d, x - center of the circle, normal is the normal to
the plane in which the circle belongs and radius of the circle can be
calculates as follows:

cosB = (a^2 + c^2 - b^2)/(2*a*c);
if cosB>1 | cosB<-1
    fprintf('There is no way to reach the end position from the given two links');
    exit(1);
end    
B = acos(cosB);
x = (a*cosB*pos)/norm(pos);
normal = pos/norm(pos);
radius = a*(sin(B));

parametric equation of the circle will be in just one parameter as it is a
curve
P(t) = rcos(t)u + rsin(t)cross(n,u) + c 

theta = acos(normal(3));
phi = asin(normal(2)/sin(theta));
u = [-sin(phi);cos(phi);0];
cross_product = cross(normal',u');
ncrossu = cross_product';

t = -pi:0.1:pi; %63 - length(t)
P = zeros(length(t),3);
for i=1:1:length(t)
    P(i,:) = (radius*cos(t(i))).*u + (radius*sin(t(i))).*ncrossu + x;
end

SO = zeros(6,3,length(P));

generate SE(3) matrix for every point of the circle
for i=1:1:length(t)
    for link 1
    x = [1;0;0];
    y = P(i,:);
    r = vrrotvec(x,y);
    R1f = generateRotationMatrix(r(1),r(2),r(3),r(4));
      
    for link 2 , from every point on the circle to the end position
    x = [1;0;0];
    y = pos;
    r = vrrotvec(x,y);
    R2f = generateRotationMatrix(r(1),r(2),r(3),r(4));
    
    SO(:,:,i) = [R1f;R2f];
end

minSOs = SO(:,:,1);
so1 = logm(inv(R1)*SO(1:3,:,1));
so2 = logm(inv(R2)*SO(4:6,:,1));

mindistance = sqrt(trace((so1.')*so1)+trace((so2.')*so2));
final = 1;

for i=2:1:length(SO)
    so1 = logm(inv(R1)*SO(1:3,:,i));
    so2 = logm(inv(R2)*SO(4:6,:,i));

    distance = sqrt(trace((so1.')*so1)+trace((so2.')*so2));
    
    if distance < mindistance
        fprintf('%d: %d\n',i,distance);
        final = i;
        mindistance = distance;
    end
end    

minSOs = SO(:,:,final);

count = 1;
[thetaTR1f, TR1f] = logSO(R1'*minSOs(1:3,:));
tempP1 = [0;0;0];
tempx1 = zeros(3,11);
for i=0:0.1:1
    [theta,S] = expso(i.*TR1f);
    tempx1(:,count) = R1*S*l1;
    vectarrow(p0,tempx1(:,count),'b');
    count = count+1;
    hold on;
end 

[thetaTR2f, TR2f] = logSO(R2'*minSOs(4:6,:));
count = 1;
for i=0:0.1:1
    [theta,S] = expso(i.*TR2f);
    tempx2 = R2*S*l2;
    tempx = tempx1(:,count);
    vectarrow(tempx, tempx+tempx2,'b');
    count = count + 1;
    hold on;
end 