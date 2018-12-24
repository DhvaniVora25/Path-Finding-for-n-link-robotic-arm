p0 = [0; 0; 0];

n=2;
fprintf('Enter the end position of the two link:\n');
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

transformations = zeros(4,4,n);

for i=1:1:n
    if i~=1
        x = links(:,i-1);
    else
        x = [norm(links(:,1));0;0];
    end
    y = links(:,i);
    r = vrrotvec(x,y);
    R = generateRotationMatrix(r(1),r(2),r(3),r(4));
    t = y-R*x;
    se = [R,t;0,0,0,1];
    transformations(:,:,i) = se;
end

fprintf('Enter the position of the end effector:\n');
position = input('');
vectarrow(p0, position, 'g');
hold on;

l1 = links(:,1)-p0;
l2 = links(:,2)-links(:,1);

a = norm(l1);
b = norm(l2);
c = norm(position);

% For the circle in 3d, x - center of the circle, normal is the normal to
% the plane in which the circle belongs and radius of the circle can be
% calculates as follows:

cosB = (a^2 + c^2 - b^2)/(2*a*c);
if cosB>1 | cosB<-1
    fprintf('There is no way to reach the end position from the given two links');
    exit(1);
end    
B = acos(cosB);
x = (a*cosB*position)/norm(position);
normal = position/norm(position);
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

SE = zeros(8,4,length(P));

%generate SE(3) matrix for every point of the circle
for i=1:1:length(t)
    % for link 1
    x = [1;0;0];
    y = P(i,:);
    r = vrrotvec(x,y);
    R = generateRotationMatrix(r(1),r(2),r(3),r(4));
    t = y'-R*x;
    se1 = [R,t;0,0,0,1];
    
    %for link 2 , from every point on the circle to the end position
    x = P(i,:);
    y = position;
    r = vrrotvec(x,y);
    R = generateRotationMatrix(r(1),r(2),r(3),r(4));
    t = y-R*x';
    se2 = [R,t;0,0,0,1];
    
    SE(:,:,i) = [se1;se2];
end

%for every point in SE, find the tangent from the starting locations of the
%link and then find the minimum of these differences.

%Then, find the geodesic from the start to the end location of the links
% links_initial is the se3*se3 element for the two links in their initial
% position, whose tangent plane is going to be considered to find the
% distance
links_initial = [transformations(:,:,1);transformations(:,:,2)];

minSEs = SE(:,:,1);
se1 = logm(inv_se(links_initial(1:4,:))*SE(1:4,:,1));
se2 = logm(inv_se(links_initial(5:8,:))*SE(5:8,:,1));

mindistance = sqrt(trace((se1.')*se1)+trace((se2.')*se2));
final = 1;

for i=2:1:length(SE)
    se1 = logm(inv_se(links_initial(1:4,:))*SE(1:4,:,i));
    se2 = logm(inv_se(links_initial(5:8,:))*SE(5:8,:,i));

    distance = sqrt(trace((se1.')*se1)+trace((se2.')*se2));
    
    if distance < mindistance
        fprintf('%d: %d\n',i,distance);
        final = i;
        minsSEs = SE(:,:,i);
        mindistance = distance;
    end
end    

minSEs = SE(:,:,final);
geodesic_se(links_initial(1:4,:),minSEs(1:4,:),[1;0;0],[0;0;0]);
geodesic_se(links_initial(5:8,:),minSEs(5:8,:),l2,l1);
