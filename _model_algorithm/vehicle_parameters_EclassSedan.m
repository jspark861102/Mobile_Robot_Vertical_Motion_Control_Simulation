function vehicle_parameters_EclassSedan

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

ms = 1653; %mb=ms;
Jy = 2765; %eyeb=Jy;
Jw = 1; %eyew=Jw;
a = 1.402; %lf=a;
b = 3.048-1.402; %lr=b; l=lf+lr;
% b = a; %lr=b; l=lf+lr;
h = 0.590;
ksf = 34000; %kf=ksf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
ksr = 46000; %kr=ksr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
% ksr = ksf; %kr=ksr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
bsf = 3264;  %cf=bsf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
bsr = 3264;  %cr=bsr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
mw = 90 * 1.2; %mwf=mw;mwr=mw; %바퀴 한개 무게
kt = 230000; ktf=kt;ktr=kt;
r = 0.359;
%M = mb+mwf+mwr;
% g = 9.81;
g = 0;
%reduction_ratio = 6.267;

bs = [-1410	-5008;
-720	-3436;
-390	-2324;
-210	-1792;
-90	-1008;
-20	-228;
20	228;
90	596;
200	784;
390	1100;
760	1796;
1160	2560;];
bs_x = bs(:,1)*0.001;
bs_y = bs(:,2);


