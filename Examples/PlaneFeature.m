function [X,Y,Z,parofplane] = PlaneFeature(px, py, pz, counter)
% NAME:
% PlaneFeature.m
% DESC:
% test to estimate the parameters of a 3D plane
% close all
% clear 
% clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% number of points
N = counter;
% inilers percentage
p = 0.8;
% noise
sigma = 0.01;

% set RANSAC options
options.epsilon = 1e-5;
options.P_inlier = 0.7;
options.sigma = sigma;
options.est_fun = @estimate_plane;
options.man_fun = @error_plane;
options.mode = 'RANSAC';
options.Ps = [];
options.notify_iters = [];
options.min_iters = counter;
options.fix_seed = false;
options.reestimate = true;
options.stabilize = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X1=zeros(3,counter);
X1(1,:)=px(:);
X1(2,:)=py(:);
X1(3,:)=pz(:);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run RANSAC
[results1, options] = RANSAC(X1, options);
disp(options);
disp(results1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Results Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a=results1.Theta(1,1);
b=results1.Theta(2,1);
c=results1.Theta(3,1);
d=results1.Theta(4,1);
normal=sqrt(a*a+b*b+c*c);

distance=zeros(1,100);
XCom=zeros(3,counter);
j=1;
k=1;
for i=1:counter
    tmp=abs(a*(px(i))+b*(py(i))+c*(pz(i))+d)/normal;
    if (tmp<1)
        distance(1,k)=tmp;
        k=k+1;
    end
    if (results1.CS(i)==1)      
        XCom(:,j)=X1(:,i);
        j=j+1;
    end
end
%plot3(handles.axes5,XCom(1,:),XCom(2,:),XCom(3,:),'sg');
%text(1,1,1,texlabel(a*x+b*y+c*z+d));
value_flatness=max(distance);
parofplane=[a,b,c,d,normal,value_flatness];
X=XCom(1,:);
Y=XCom(2,:);
Z=XCom(3,:);
fprintf('\nThe Flatness about the Plane is %d\n',value_flatness);
return











