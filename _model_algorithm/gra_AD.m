function dWd_p = gra_AB(t,Wd_p)

load A.mat;
load D.mat;
load Sw.mat;


L = size(A);
L = L(1,1)*L(1,2);

Wd = zeros(length(A),length(A));

for i = 1 : length(A)
    for j = 1 : length(A)
        Wd(i,j) = Wd_p(j+(i-1)*length(A));
    end
end
    
dWd = A*Wd + Wd*A' + D*Sw*D';

dWd_p = zeros(L,1);
for i = 1 : L    
    dWd_p(i) = dWd(floor((i-1)/length(A))+1,rem((i-1),length(A))+1);    
end
