function [E T_noise_squared d] = error_PointCloudCircle(Theta, X, sigma, P_inlier, parameters)

% [E T_noise_squared d] = error_line(Theta, X, sigma, P_inlier, parameters)
%
% DESC:
% estimate the squared fitting error for a cricle expresed in cartesian form
% a(X.^2 + Y.^2 +Z.^2) + bX + cY + dZ + e =0
%
% AUTHOR
% Marco Zuliani - marco.zuliani@gmail.com
%
% VERSION:
% 1.0.0
%
% INPUT:
% Theta             = line parameter vector
% X                 = samples on the manifold
% sigma             = noise std
% P_inlier          = Chi squared probability threshold for inliers
%                     If 0 then use directly sigma.
%
% OUTPUT:
% E                 = squared error 
% T_noise_squared   = noise threshold
% d                 = degrees of freedom of the error distribution

% HISTORY
%
% 1.0.0             - 01/26/08 initial version
% 1.0.1             - 02/22/09 updated error threshold

% compute the squared error
E = [];
if ~isempty(Theta) && ~isempty(X)
    % centroid = mean([transpose(X(1,:)) transpose(X(2,:))]);
    % Par = [a b R] is the fitting circle:
    % center (a,b) and radius R
    % Par = [-(Theta(2:3))'/Theta(1)/2 , sqrt(Theta(2)*Theta(2)+Theta(3)*Theta(3)-4*Theta(1)*Theta(4))/abs(Theta(1))/2];
    a=Theta(1);
    b=Theta(2);
    c=Theta(3);
    d=Theta(4);
    e=Theta(5);
    r=sqrt(0.25*(b^2/a^2+c^2/a^2+d^2/a^2)-e/a);
    %disp(r);
    E = abs(r - sqrt((X(1,:) + b/(2*a)).^2 + (X(2,:) + c/(2*a)).^2 + (X(3,:) + d/(2*a)).^2));
    % den = Theta(1)^2 + Theta(2)^2 + Theta(3)^2 + Theta(4)^2;
    %
    % E = ( Theta(1)*(X(1,:).*X(1,:) + X(2,:).*X(2,:)) + Theta(2)*X(1,: ) + Theta(3)*X(2,:) +Theta(4)).^2 / den;
                
end;

% compute the error threshold
if (nargout > 1)
    
    if (P_inlier == 0)
        T_noise = sigma;
    else
        % Assumes the errors are normally distributed. Hence the sum of
        % their squares is Chi distributed (with 2 DOF since we are 
        % computing the distance of a 2D point to a line)
        d = 2;
        
        % compute the inverse probability
        T_noise_squared = sigma^2 * chi2inv_LUT(P_inlier, d);

    end;
    
end;

return;
