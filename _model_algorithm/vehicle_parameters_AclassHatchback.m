function vehicle_parameters_AclassHatchback

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

ms = 747; %mb=ms;
Jy = 1110.9; %eyeb=Jy;
Jw = 1; %eyew=Jw;
a = 1.103; %lf=a;
b = 2.347-1.103; %lr=b; l=lf+lr;
h = 0.540;
ksf = 14000; %kf=ksf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
ksr = 18000; %kr=ksr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
bsf = 1723;  %cf=bsf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
bsr = 1723;  %cr=bsr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
mw = 41.5 * 1.2; %mwf=mw;mwr=mw; %바퀴 한개 무게
kt = 200000; ktf=kt;ktr=kt;
r = 0.292;
%M = mb+mwf+mwr;
g = 9.81;
% g = 0;
%reduction_ratio = 6.267;


bs=[-1410	-2644;
-720	-1814;
-390	-1227;
-210	-946;
-90	-532;
-20	-120;
20	120;
90	314;
200	413;
390	580;
760	948;
1160	1351;];
bs_x = bs(:,1)*0.001;
bs_y = bs(:,2);

