function dWc_p = gra_AB(t,Wc_p)

load A.mat;
load B.mat

L = size(A);
L = L(1,1)*L(1,2);

Wc = zeros(length(A),length(A));

for i = 1 : length(A)
    for j = 1 : length(A)
        Wc(i,j) = Wc_p(j+(i-1)*length(A));
    end
end
    
dWc = A*Wc + Wc*A' + B*B';

dWc_p = zeros(L,1);
for i = 1 : L    
    dWc_p(i) = dWc(floor((i-1)/length(A))+1,rem((i-1),length(A))+1);    
end
