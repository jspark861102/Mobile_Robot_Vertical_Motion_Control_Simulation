function dWd_p = gra_AB(t,Wd_p)

load Ar.mat;
load Dr.mat;
load Sw.mat;

L = size(Ar);
L = L(1,1)*L(1,2);

Wd = zeros(length(Ar),length(Ar));

for i = 1 : length(Ar)
    for j = 1 : length(Ar)
        Wd(i,j) = Wd_p(j+(i-1)*length(Ar));
    end
end
    
dWd = Ar*Wd + Wd*Ar' + Dr*Sw*Dr';

dWd_p = zeros(L,1);
for i = 1 : L    
    dWd_p(i) = dWd(floor((i-1)/length(Ar))+1,rem((i-1),length(Ar))+1);    
end
