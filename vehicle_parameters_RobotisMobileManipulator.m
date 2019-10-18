function vehicle_parameters_RobotisMobileManipulator

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

ms = 10;  %lincluding suspension
mw = 0.6; %���� �Ѱ� ����
a = 0.10;
b = 0.16;
r = 0.075;
Jy = 1/12*ms*(a^2+b^2); %1/12*ms*(a^2+b^2)
Jw = 1; %1/2*mw*r^2
h = 0.05;
ksf = 5*9.8/0.01; %kf=ksf; %���� ������ ��, ���� �� ǥ���ҷ��� ���ϱ� 2
ksr = 5*9.8/0.01; %kr=ksr; %���� ������ ��, ���� �� ǥ���ҷ��� ���ϱ� 2
bsf = 10;  %cf=bsf; %���� ������ ��, ���� �� ǥ���ҷ��� ���ϱ� 2
bsr = 10;  %cr=bsr; %���� ������ ��, ���� �� ǥ���ҷ��� ���ϱ� 2
kt = 200000; ktf=kt;ktr=kt;
g = 9.81;