function y=DH(theta,d,a,alfa)
Rot1=[cos(theta) -sin(theta) 0 0;
    sin(theta) cos(theta) 0 0;
    0 0 1 0;
    0 0 0 1];
Trans1=[1 0 0 0;
        0 1 0 0;
        0 0 1 d;
        0 0 0 1];
Trans2=[1 0 0 a;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];
Rot2=[1 0 0 0;
      0 cos(alfa) -sin(alfa) 0;
      0 sin(alfa) cos(alfa) 0;
      0 0 0 1];   
y=Rot1*Trans1*Trans2*Rot2;
end