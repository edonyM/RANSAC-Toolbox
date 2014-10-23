% NAME:
% PointCloudPlane_test.m
%
% DESC:
% test to estimate the parameters of a 3D plane

close all
%clear 
% clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% number of points
N = couter1;
% inilers percentage
p = 0.25;
% noise
sigma = 0.001;

% set RANSAC options
options.epsilon = 1e-6;
options.P_inlier = 0.99;
options.sigma = sigma;
options.est_fun = @estimate_plane;
options.man_fun = @error_plane;
options.mode = 'RANSAC';
options.Ps = [];
options.notify_iters = [];
options.min_iters = couter1;
options.fix_seed = false;
options.reestimate = true;
options.stabilize = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%random select three point from the point cloud
%i1=randi(couter1);
%i2=randi(couter1);
%i3=randi(couter1);
%i4=randi(couter1);
%X_pct1=[PartX1(i1) PartX1(i2) PartX1(i3) PartX1(i4)];
%Y_pct1=[PartY1(i1) PartY1(i2) PartY1(i3) PartY1(i4)];
%Z_pct1=[PartZ1(i1) PartZ1(i2) PartZ1(i3) PartZ1(i4)];
%v1 = [PartX1(i2)-PartX1(i1);PartY1(i2)-PartY1(i1);PartZ1(i2)-PartZ1(i1)];
%v1 = v1/norm(v1);
%v2 = [PartX1(i3)-PartX1(i2);PartY1(i3)-PartY1(i2);PartZ1(i3)-PartZ1(i2)];
%v2 = v2/norm(v2);
%X0 = [PartX1(i4);PartY1(i4);PartZ1(i4)];

X1=zeros(3,couter1);
X1(1,:)=PartX1(:);
X1(2,:)=PartY1(:);
X1(3,:)=PartZ1(:);

X2=zeros(3,couter2);
X2(1,:)=PartX2(:);
X2(2,:)=PartY2(:);
X2(3,:)=PartZ2(:);

X3=zeros(3,couter3);
X3(1,:)=PartX3(:);
X3(2,:)=PartY3(:);
X3(3,:)=PartZ3(:);

X4=zeros(3,couter4);
X4(1,:)=PartX4(:);
X4(2,:)=PartY4(:);
X4(3,:)=PartZ4(:);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run RANSAC
[results1, options] = RANSAC(X1, options);
[results2, options] = RANSAC(X2, options);
[results3, options] = RANSAC(X3, options);
[results4, options] = RANSAC(X4, options);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Results Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
plot3(PartX1,PartY1,PartZ1,'.r');
hold on
couter=1;
Xp=zeros(1,100);
Yp=zeros(1,100);
Zp=zeros(1,100);
for ic=1:couter1
    if (results1.CS(ic)==1)
        Xp(couter)=PartX1(ic);
        Yp(couter)=PartY1(ic);
        Zp(couter)=PartZ1(ic);
        couter=couter+1;
    end
end
plot3(Xp,Yp,Zp,'sg');

figure(2)
plot3(PartX2,PartY2,PartZ2,'.r');
hold on
couter=1;
Xp=zeros(1,100);
Yp=zeros(1,100);
Zp=zeros(1,100);
for ic=1:couter2
    if (results2.CS(ic)==1)
        Xp(couter)=PartX2(ic);
        Yp(couter)=PartY2(ic);
        Zp(couter)=PartZ2(ic);
        couter=couter+1;
    end
end
plot3(Xp,Yp,Zp,'sg');

figure(3)
plot3(PartX3,PartY3,PartZ3,'.r');
hold on
couter=1;
Xp=zeros(1,100);
Yp=zeros(1,100);
Zp=zeros(1,100);
for ic=1:couter3
    if (results3.CS(ic)==1)
        Xp(couter)=PartX3(ic);
        Yp(couter)=PartY3(ic);
        Zp(couter)=PartZ3(ic);
        couter=couter+1;
    end
end
plot3(Xp,Yp,Zp,'sg');

figure(4)
plot3(PartX4,PartY4,PartZ4,'.r');
hold on
couter=1;
Xp=zeros(1,100);
Yp=zeros(1,100);
Zp=zeros(1,100);
for ic=1:couter4
    if (results4.CS(ic)==1)
        Xp(couter)=PartX4(ic);
        Yp(couter)=PartY4(ic);
        Zp(couter)=PartZ4(ic);
        couter=couter+1;
    end
end
plot3(Xp,Yp,Zp,'sg');














