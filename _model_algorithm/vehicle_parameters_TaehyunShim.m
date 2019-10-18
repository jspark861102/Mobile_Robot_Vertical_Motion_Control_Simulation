function vehicle_parameters_TaehyunShim

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

ms = 1440; %mb=ms;
Jy = 2000; %eyeb=Jy;
Jw = 1; %eyew=Jw;
a = 1.016; %lf=a;
b = 1.524; %lr=b; l=lf+lr;
h = 0.465;
ksf = 35000; %kf=ksf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
ksr = 30000; %kr=ksr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
bsf = 2500;  %cf=bsf; %앞쪽 한쪽의 값, 앞쪽 다 표현할려면 곱하기 2
bsr = 2000;  %cr=bsr; %뒤쪽 한쪽의 값, 뒤쪽 다 표현할려면 곱하기 2
mw = 80; %mwf=mw;mwr=mw; %바퀴 한개 무게
kt = 200000; ktf=kt;ktr=kt;
r = 0.285;
%M = mb+mwf+mwr;
% g = 9.81;
g = 0;
%reduction_ratio = 6.267;