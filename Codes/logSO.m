function [theta,S] = logSO(R)
theta = acosd((trace(R)-1)/2);
temp = 1/2*sin(theta);
%S = temp.*(R-transpose(R));
S = logm(R);