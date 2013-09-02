%projected normal function: directional statistics Page46
mean1 = 1.0; mean2 = 1.0;
u = [mean1; mean2];
cov1 = 0.2; cov2 = 0.2;
COV = [cov1, 0.0; 0.0, cov2];

interval = 0.01;
thetha = [0:interval:(2*pi-interval)];

[tmp, num] = size(thetha);
p = zeros(1, num);

for i = 1:num
    x =[cos(thetha(i)); sin(thetha(i))];
    phi = mvnpdf([mean1 mean2], 0, COV);
    D_thetha = (u'/COV *x) /sqrt(x'/COV*x);
    PHI_D_thetha = normcdf(D_thetha, 0, 1);
    phi_factor = 1/(sqrt(det(COV)*sqrt(x'/COV*x)))*(mean1*sin(thetha(i))-mean2*cos(thetha(i)));
    phi_factor_value = normpdf(phi_factor, 0, 1);
    p(i)=(phi+1/sqrt(det(COV))*D_thetha*PHI_D_thetha*phi_factor_value)/(x'/COV*x);    
end

plot(thetha, p)
hold on;