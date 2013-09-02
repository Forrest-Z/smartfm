%projected normal function: directional statistics Page46
function [mean_thetha, thetha_variance] = projected_normal_function(mean1, mean2, cov1, cov2)

%tic

%mean1 = 1; mean2 = 1;
u = [mean1; mean2];
%cov1 = 0.2; cov2 = 0.2;
COV = [cov1, 0.0; 0.0, cov2];

interval = 0.01;
thetha = [0:interval:(2*pi-interval)];

[tmp, num] = size(thetha);
p = zeros(1, num);
p_sum = 0;

mean_value  = 0;
mean_thetha = 0;

for i = 1:num
    x =[cos(thetha(i)); sin(thetha(i))];
    phi = mvnpdf([mean1 mean2], 0, COV);
    D_thetha = (u'/COV *x) /sqrt(x'/COV*x);
    PHI_D_thetha = normcdf(D_thetha, 0, 1);
    phi_factor = 1/(sqrt(det(COV)*sqrt(x'/COV*x)))*(mean1*sin(thetha(i))-mean2*cos(thetha(i)));
    phi_factor_value = normpdf(phi_factor, 0, 1);
    p(i)=(phi+1/sqrt(det(COV))*D_thetha*PHI_D_thetha*phi_factor_value)/(x'/COV*x);    
    
    if p(i)>mean_value
        mean_value = p(i);
        mean_thetha = thetha(i);
    end
    p_sum = p(i) + p_sum;
end

%plot(thetha, p)

%calculate the variance;
thetha_variance = 0;

for i=1:num
    %normalize the weight
    p(i)=p(i)/p_sum;
    delta_thetha =  abs(thetha(i)-mean_thetha);
    if delta_thetha > pi
        delta_thetha = pi*2 - delta_thetha;
    end
    thetha_variance = p(i)*delta_thetha*delta_thetha +thetha_variance;
end

%toc
%hold on;
end