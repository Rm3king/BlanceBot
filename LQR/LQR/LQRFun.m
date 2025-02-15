function [K] = LQRFun(basicdata, valdata, MatQ, MatR)

g = basicdata(1);
M = basicdata(2);
R = basicdata(3);
l = basicdata(4);
mw = basicdata(5);
mp = basicdata(6);
Iw = basicdata(7);


%%可变值
L  = valdata(1);
Lm = valdata(2);      
Im = valdata(3);
Ip = valdata(4);

A1 = (2*Im*Iw*L*g*mp + 2*Im*L*M^2*R^2*g + 2*Im*Lm*M^2*R^2*g + 2*Iw*L*M^2*g*l^2 + 2*Iw*Lm*M^2*g*l^2 + 2*Im*L*R^2*g*mp^2 + 2*Im*Iw*L*M*g + 2*Im*Iw*Lm*M*g + 2*L*M*R^2*g*l^2*mp^2 + 2*L*M^2*R^2*g*l^2*mp + 2*L*M^2*R^2*g*l^2*mw + 2*Lm*M^2*R^2*g*l^2*mp + 2*Lm*M^2*R^2*g*l^2*mw + 4*Im*L*M*R^2*g*mp + 2*Im*L*M*R^2*g*mw + 2*Im*Lm*M*R^2*g*mp + 2*Im*Lm*M*R^2*g*mw + 2*Iw*L*M*g*l^2*mp + 2*Im*L*R^2*g*mp*mw + 2*L*M*R^2*g*l^2*mp*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A2 = (2*Iw*L*M^2*g*l^2 + 2*Iw*Lm*M^2*g*l^2 + 2*L*M^2*R^2*g*l^2*mw + 2*Lm*M^2*R^2*g*l^2*mp + 2*Lm*M^2*R^2*g*l^2*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A3 = (2*g*mw*L^2*M^2*R^2*l^2 + 2*Iw*g*L^2*M^2*l^2 + 2*g*mw*L^2*M*R^2*l^2*mp + 2*g*mw*Im*L^2*M*R^2 + 2*Iw*g*L^2*M*l^2*mp + 2*Iw*g*Im*L^2*M + 2*g*mw*Im*L^2*R^2*mp + 2*Iw*g*Im*L^2*mp + 2*g*L*Lm*M^2*R^2*l^2*mp + 4*g*mw*L*Lm*M^2*R^2*l^2 + 4*Iw*g*L*Lm*M^2*l^2 + 2*g*L*Lm*M*R^2*l^2*mp^2 + 2*g*mw*L*Lm*M*R^2*l^2*mp + 2*g*Im*L*Lm*M*R^2*mp + 4*g*mw*Im*L*Lm*M*R^2 + 2*Iw*g*L*Lm*M*l^2*mp + 4*Iw*g*Im*L*Lm*M + 2*g*Im*L*Lm*R^2*mp^2 + 2*g*mw*Im*L*Lm*R^2*mp + 2*Iw*g*Im*L*Lm*mp + 2*g*Lm^2*M^2*R^2*l^2*mp + 2*g*mw*Lm^2*M^2*R^2*l^2 + 2*Iw*g*Lm^2*M^2*l^2 + 2*g*Im*Lm^2*M*R^2*mp + 2*g*mw*Im*Lm^2*M*R^2 + 2*Iw*g*Im*Lm^2*M)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A4 = (2*Iw*L^2*M^2*g*l^2 + 2*Iw*Lm^2*M^2*g*l^2 + 2*Ip*M^2*R^2*g*l^2 + 2*L^2*M^2*R^2*g*l^2*mw + 2*Lm^2*M^2*R^2*g*l^2*mp + 2*Lm^2*M^2*R^2*g*l^2*mw + 4*Iw*L*Lm*M^2*g*l^2 + 4*L*Lm*M^2*R^2*g*l^2*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A5 = (2*g*l*mw*L^2*M^2*R^2 + 2*Iw*g*l*L^2*M^2 + 2*g*l*mw*L^2*M*R^2*mp + 2*Iw*g*l*L^2*M*mp + 2*g*l*L*Lm*M^2*R^2*mp + 4*g*l*mw*L*Lm*M^2*R^2 + 4*Iw*g*l*L*Lm*M^2 + 2*g*l*L*Lm*M*R^2*mp^2 + 2*g*l*mw*L*Lm*M*R^2*mp + 2*Iw*g*l*L*Lm*M*mp + 2*g*l*Lm^2*M^2*R^2*mp + 2*g*l*mw*Lm^2*M^2*R^2 + 2*Iw*g*l*Lm^2*M^2)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A6 = (2*Ip*Iw*M*g*l + 2*Iw*L^2*M^2*g*l + 2*Iw*Lm^2*M^2*g*l + 2*Ip*M^2*R^2*g*l + 2*L^2*M^2*R^2*g*l*mw + 2*Lm^2*M^2*R^2*g*l*mp + 2*Lm^2*M^2*R^2*g*l*mw + 4*Iw*L*Lm*M^2*g*l + 2*Iw*L^2*M*g*l*mp + 2*Ip*M*R^2*g*l*mp + 2*Ip*M*R^2*g*l*mw + 4*L*Lm*M^2*R^2*g*l*mw + 2*L^2*M*R^2*g*l*mp*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);

B1 = -(2*Im*Iw + 2*Im*M*R^2 + 2*Iw*M*l^2 + 2*Im*R^2*mp + 2*Im*R^2*mw + 2*Im*L*M*R + 2*Im*Lm*M*R + 2*M*R^2*l^2*mp + 2*M*R^2*l^2*mw + 2*Im*L*R*mp + 2*L*M*R*l^2*mp)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B2 = (2*Im*Iw + 2*Im*M*R^2 + 2*Iw*M*l^2 + 2*Im*R^2*mp + 2*Im*R^2*mw + 2*M*R^2*l^2*mp + 2*M*R^2*l^2*mw + 2*Iw*L*M*l + 2*Iw*Lm*M*l + 2*L*M*R^2*l*mw + 2*Lm*M*R^2*l*mp + 2*Lm*M*R^2*l*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B3 = -(2*Im*Iw*L + 2*Im*Iw*Lm - 2*Im*Ip*R + 2*Iw*L*M*l^2 + 2*Iw*Lm*M*l^2 - 2*Ip*M*R*l^2 + 2*Im*L*R^2*mw + 2*Im*Lm*R^2*mp + 2*Im*Lm*R^2*mw + 2*Im*L*Lm*R*mp + 2*L*M*R^2*l^2*mw + 2*Lm*M*R^2*l^2*mp + 2*Lm*M*R^2*l^2*mw + 2*L*Lm*M*R*l^2*mp)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B4 = (2*Im*Iw*L + 2*Im*Iw*Lm + 2*Iw*L*M*l^2 + 2*Iw*L^2*M*l + 2*Iw*Lm*M*l^2 + 2*Iw*Lm^2*M*l + 2*Ip*M*R^2*l + 2*Im*L*R^2*mw + 2*Im*Lm*R^2*mp + 2*Im*Lm*R^2*mw + 2*L*M*R^2*l^2*mw + 2*L^2*M*R^2*l*mw + 2*Lm*M*R^2*l^2*mp + 2*Lm^2*M*R^2*l*mp + 2*Lm*M*R^2*l^2*mw + 2*Lm^2*M*R^2*l*mw + 4*Iw*L*Lm*M*l + 4*L*Lm*M*R^2*l*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B5 = -(2*Iw*L*M*l + 2*Iw*Lm*M*l - 2*Ip*M*R*l + 2*L*M*R^2*l*mw + 2*Lm*M*R^2*l*mp + 2*Lm*M*R^2*l*mw + 2*L*Lm*M*R*l*mp)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B6 =(2*Ip*Iw + 2*Iw*L^2*M + 2*Iw*Lm^2*M + 2*Ip*M*R^2 + 2*Iw*L^2*mp + 2*Ip*R^2*mp + 2*Ip*R^2*mw + 2*L^2*M*R^2*mw + 2*Lm^2*M*R^2*mp + 2*Lm^2*M*R^2*mw + 4*Iw*L*Lm*M + 2*L^2*R^2*mp*mw + 2*Iw*L*M*l + 2*Iw*Lm*M*l + 4*L*Lm*M*R^2*mw + 2*L*M*R^2*l*mw + 2*Lm*M*R^2*l*mp + 2*Lm*M*R^2*l*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A = [0 1 0 0 0 0; A1 0 0 0 A2 0; 0 0 0 1 0 0; A3 0 0 0 A4 0; 0 0 0 0 0 1; A5 0 0 0 A6 0];
B = [0 0; B1 B2; 0 0; B3 B4; 0 0; B5 B6];

[v,d]=eig(A);
D=[B,A*B,A*A*B,A*A*A*B,A*A*A*A*B,A*A*A*A*A*B];
r1=rank(D);

C = eye(6,6);
D = eye(6,2);
E=[C;C*A;C*A*A;C*A*A*A;C*A*A*A*A;C*A*A*A*A*A];
r2=rank(E);

K=lqr(A,B,MatQ,MatR); %调用 lqr 函数用以求解状态反馈矩阵 K
end