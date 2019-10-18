function dP_p = ricca(t,P_p)

load A.mat;
load B.mat;
load Q.mat;

L = size(A);
L = L(1,1)*L(1,2);

P = zeros(length(A),length(A));

for i = 1 : length(A)
    for j = 1 : length(A)
        P(i,j) = P_p(j+(i-1)*length(A));
    end
end
    
dP = -1*(A'*P + P*A + Q - P*B*B'*P);

dP_p = zeros(L,1);
for i = 1 : L    
    dP_p(i) = dP(floor((i-1)/length(A))+1,rem((i-1),length(A))+1);    
end
