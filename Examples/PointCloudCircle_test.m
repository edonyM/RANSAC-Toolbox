% NAME:
% PointCloud_test.m
%
% DESC:
% test to estimate the parameters of a 3D plane

close all
clear all 
%clc
matr_of_stl=importdata('out.txt');
[B,ii,jj]=unique(matr_of_stl,'rows');
[m,n]=size(B);
cloud=zeros(n,m);
for i=1:m;
    for j=1:n;
        cloud(j,i)=B(i,j);
    end
end
x=cloud(1,:);
y=cloud(2,:);
z=cloud(3,:);
%plot3(x,y,z,'.m')
%grid minor
%axis square

%%
%get the part of the graph pictured by point-cloud
%%the whole view from the direction of x,y,z
%subplot(7,2,1);plot(x,y,'.r')
%subplot(7,2,2);plot(x,z,'.r')
%subplot(7,2,3);plot(y,z,'.r')


%%
%the first base
%
couter1=0;
I1=zeros(1,100);
for i1=1:m
    if(z(i1)>1700&&z(i1)<1725)%if(z(i1)>101&&z(i1)<104)
        couter1=couter1+1;
        I1(couter1)=i1;
    end
end
x1=zeros(1,couter1);
y1=zeros(1,couter1);
j1=1;
for i1=1:couter1
        x1(j1)=x(I1(i1));
        y1(j1)=y(I1(i1));
        j1=j1+1;
end
%subplot(4,2,1);plot(x1,y1,'.r')
PartX1=zeros(1,10);
PartY1=zeros(1,10);
PartZ1=zeros(1,10);
for pic=1:couter1
PartX1(pic)=x(I1(pic));
PartY1(pic)=y(I1(pic));
PartZ1(pic)=z(I1(pic));
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% number of points
N = couter1;
% inilers percentage
p = 0.9;
% noise
sigma = 0.15;

% set RANSAC options
options.epsilon = 1e-6;
options.P_inlier = 0.99;
options.sigma = sigma;
options.est_fun = @estimate_PointCloudCircle;
options.man_fun = @error_PointCloudCircle;
options.mode = 'RANSAC';
options.Ps = [];
options.notify_iters = [];
options.min_iters = 1000;
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

X1=zeros(2,couter1);
X1(1,:)=PartX1(:);
X1(2,:)=PartY1(:);
%X1(3,:)=PartZ1(:);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run RANSAC
[results1, options] = RANSAC(X1, options);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Results Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
plot(PartX1,PartY1,'.r');
hold on
couter=1;
Xp=zeros(1,100);
Yp=zeros(1,100);
for ic=1:couter1
    if (results1.CS(ic)==1)
        Xp(couter)=PartX1(ic);
        Yp(couter)=PartY1(ic);
        couter=couter+1;
    end
end
plot(Xp,Yp,'sg');
