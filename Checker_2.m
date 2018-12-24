plot3(P(:,1),P(:,2),P(:,3))
hold on
vectarrow([0;0;0], pos, 'g');
hold on
vectarrow([0;0;0],[ptmin1(1);ptmin1(2);ptmin1(3)],'r')
hold on
vectarrow([ptmin1(1);ptmin1(2);ptmin1(3)],[ptmin2(1);ptmin2(2);ptmin2(3)],'b')
hold on
vectarrow([0;0;0],l1,'b');
hold on
vectarrow(l1,l1+l2,'r');
hold on