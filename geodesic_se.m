function i = geodesic_se(SE1,SE2,p,p0)
i=1;


x1 = SE1*[p;1];
p1 = x1(1:3);

t1 = SE1(1:3,4);
t2 = SE2(1:3,4);

TR2Id = logm(inv_se(SE1)*SE2);
TR2 = TR2Id;

for i=0:0.1:1
    R = expm(i.*TR2);
    tempP1 = SE1*(R*[p1;1]);
    vectarrow(p0+t1+(i*(t2-t1)),[tempP1(1,1);tempP1(2,1);tempP1(3,1)],'r');
    hold on;
end    