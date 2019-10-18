function dWc_p = gra_ArBr(t,Wc_p)

load Ar.mat;
load Br.mat

L = size(Ar);
L = L(1,1)*L(1,2);

Wc = zeros(length(Ar),length(Ar));

for i = 1 : length(Ar)
    for j = 1 : length(Ar)
        Wc(i,j) = Wc_p(j+(i-1)*length(Ar));
    end
end
    
dWc = Ar*Wc + Wc*Ar' + Br*Br';

dWc_p = zeros(L,1);
for i = 1 : L    
    dWc_p(i) = dWc(floor((i-1)/length(Ar))+1,rem((i-1),length(Ar))+1);    
end
