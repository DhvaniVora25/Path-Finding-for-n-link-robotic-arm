fprintf('Enter final position of link 1');
link1_init = input('');

vectarrow([0;0;0],link1_init);
hold on;
l1 = norm(link1_init);
%link1 = [l1;0;0];
%R11 = vrrotvec(link1,link1_init);
%R1 = generateRotationMatrix(R11(1),R11(2),R11(3),R11(4));
t = link1_init;

fprintf('Enter final position of link 2');
link2_init = input('');
l2 = norm(link2_init-link1_init);
link2 = [l2;0;0];
R21 = vrrotvec(link2,link2_init-link1_init);
R2 = generateRotationMatrix(R21(1),R21(2),R21(3),R21(4));

SE2 = [R2,t;0,0,0,1];

link2_init1 = SE2*[link2;1];

vectarrow(link1_init, link2_init);
hold on;

fprintf('Enter the final position of the end effector:');
pos = input('');

vectarrow([0;0;0],pos);
hold on;

l1I = link1_init;
l2I = link2_init-link1_init;

a = norm(l1I);
b = norm(l2I);
c = norm(pos);

% For the circle in 3d, x - center of the circle, normal is the normal to
% the plane in which the circle belongs and radius of the circle can be
% calculates as follows:

cosB = (a^2 + c^2 - b^2)/(2*a*c);
if cosB>1 | cosB<-1
    fprintf('There is no way to reach the end position from the given two links');
    exit(1);
end    
B = acos(cosB);
x = (a*cosB*pos)/norm(pos);
normal = pos/norm(pos);
radius = a*(sin(B));

%parametric equation of the circle will be in just one parameter as it is a
%curve
% P(t) = rcos(t)u + rsin(t)cross(n,u) + c 

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

SE = zeros(4,4,length(P));

%generate SE(3) matrix for every point of the circle
for i=1:1:length(t)
    % for link 1
    t = P(i,:).';
      
    %for link 2 , from every point on the circle to the end position
    x = [l2;0;0];
    y = pos-P(i,:).';
    r = vrrotvec(x,y);
    R2f = generateRotationMatrix(r(1),r(2),r(3),r(4));
    
    SE(:,:,i) = [R2f,t;0,0,0,1];
end

SE_temp = logm(inv_se(SE2)*SE(:,:,1));

mindistance = sqrt(trace((SE_temp.')*SE_temp));
final = 1;

for i=2:1:length(SE)
    SE_temp = logm(inv_se(SE2)*SE(:,:,i));
    
    distance = sqrt(trace((SE_temp.')*SE_temp));
    
    if distance < mindistance
        %fprintf('%d: %d\n',i,distance);
        final = i;
        mindistance = distance;
    end
end    

minSE = SE(:,:,final);

TR1 = logm(inv_se(SE2)*minSE);
tempP1 = [0;0;0];

count = 1;
for i=0:0.1:1
    S = expm(i.*TR1);
    tempx2 = SE2*S*[link2;1];
    tempx21 = tempx2(1:3,1);
    tempx = S(1:3,4);
    vectarrow([0;0;0], tempx);
    hold on;
    vectarrow(tempx,tempx21);
    hold on;
    count = count + 1;
    hold on;
end 