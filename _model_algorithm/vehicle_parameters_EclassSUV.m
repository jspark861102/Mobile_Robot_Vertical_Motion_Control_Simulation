function vehicle_parameters_EclassSUV

global ms; 
global Jy;
global Jw;
global a;
global b;
global h;
global ksf;
global ksr;
global bsf
global bsr;
global mw;
global kt;
global ktf;
global ktr;
global r;
global g;

ms = 1592; %mb=ms;
Jy = 2488; %eyeb=Jy;
Jw = 1; %eyew=Jw;
a = 1.180; %lf=a;
b = 2.950-1.180; %lr=b; l=lf+lr;
h = 0.719;
ksf = 146000; %kf=ksf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
ksr = 46000; %kr=ksr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
bsf = 4768;  %cf=bsf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
bsr = 4768;  %cr=bsr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
mw = 135 * 1.2; %mwf=mw;mwr=mw; %바퀴 한개 무게
kt = 230000; ktf=kt;ktr=kt;
r = 0.380;
%M = mb+mwf+mwr;
% g = 9.81;
g = 0;
%reduction_ratio = 6.267;


bs=[-1410	-7316;
-720	-5019;
-390	-3395;
-210	-2618;
-90	-1472;
-20	-333;
20	333;
90	870;
200	1145;
390	1607;
760	2623;
1160	3740;
];
bs_x = bs(:,1)*0.001;
bs_y = bs(:,2);

