function [CS, Para, R, Center]=CircleTest(px, py, pz, couter, radius)
% NAME:
% CircleTest.m
%
% DESC:
% test to estimate the parameters of a 3D circle

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% number of points
N = couter;
% inilers percentage
p = 0.7;
% noise
sigma = 0.1;

% set RANSAC options
options.epsilon = 1e-6;
options.P_inlier = 0.8;
options.sigma = sigma;
options.est_fun = @estimate_PointCloudCircle;
options.man_fun = @error_PointCloudCircle;
options.mode = 'MSAC';
options.Ps = [];
options.notify_iters = [];
options.min_iters = couter;
options.fix_seed = false;
options.reestimate = true;
options.stabilize = false;

% here we set theradius of the circle that we want to detect
options.parameters.radius = radius;
%disp(options.parameters.radius);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%random select three point from the point cloud
X1=zeros(3,couter);
X1(1,:)=px(:);
X1(2,:)=py(:);
X1(3,:)=pz(:);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run RANSAC
[results1, ~] = RANSAC(X1, options);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Results Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a=results1.Theta(1,1);
b=results1.Theta(2,1);
c=results1.Theta(3,1);
d=results1.Theta(4,1);
e=results1.Theta(5,1);
R=sqrt((1/(4*(a*a)))*(b*b + c*c + d*d) - e/a);
Center=[-b/2*a,-c/2*a,-d/2*a];
Para=[a,b,c,d,e];
CS=results1.CS(:);
%disp(Para);
return

























