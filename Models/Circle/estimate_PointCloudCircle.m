function [Theta, k] = estimate_PointCloudCircle(X, s, parameters)

% [Theta k] = estimate_foo(X, s, parameters)
%
% DESC:
% Template for the estimation function to be used inside RANSAC
%
% INPUT:
% X                 = 2D point correspondences
% s                 = indices of the points used to estimate the 
%                     parameter vector. If empty all the points 
%                     are used
%
% parameters        = parameters.radius is the radius of the circle
%
% OUTPUT:
% Theta             = estimated parameter vector Theta = H(:). If  
%                     the estimation is not successful return an 
%                     empty vector. i.e. Theta = [];
% k                 = dimension of the minimal subset

% here we define the size of the MSS
k = 3;

% check if the input parameters are valid
if (nargin == 0) || isempty(X)
    Theta = [];
    return;
end;

% select the points to estimate the parameter vector
if (nargin == 2) && ~isempty(s)
    X = X(:, s);
end;

% check if we have enough points
N = size(X, 2);
if (N < k)
    error('estimate_foo:inputError', ...
        'At least 3 point correspondences are required');
end;

% call the estimation function foo
% a*X^2+b*Y^2+c*X+d*Y+e=0
A = [transpose(X(1, :).*X(1,:))+transpose(X(2, :).*X(2,:))+transpose(X(3, :).*X(3,:)) transpose(X(1,:)) transpose(X(2,:)) transpose(X(3,:)) ones(N, 1)];
[~, ~, V] = svd(A);
rad=parameters.radius;
j=1;
for i=1:5
    th=V(:,i);
    a=th(1);
    b=th(2);
    c=th(3);
    d=th(4);
    e=th(5);
    tmp = 0.25*(b^2 + c^2 + d^2)/a^2 - e/a;
    if tmp>0
        %tmp2 = abs(tmp - rad);
        j=j+1;
    end   
end
r=zeros(2,j-1);
k=1;
for i=1:5
    th=V(:,i);
    a=th(1);
    b=th(2);
    c=th(3);
    d=th(4);
    e=th(5);
    tmp = 0.25*(b^2 + c^2 + d^2)/a^2 - e/a;
    if tmp>0
        %tmp2 = abs(tmp - rad);
        r(1,k)=abs(tmp-rad^2);
        r(2,k)=i;
        k=k+1;
    end   
end
for i=1:j-1
    if r(1,i)==min(r(1,:))
        tc=r(2,i);
    end
end
Theta = V(:,tc);
disp(V)
% here you may want to check if the results are meaningful.
% If not return an empty vector
