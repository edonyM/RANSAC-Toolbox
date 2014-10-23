function [parallelvalue,res]=ParallismTest(baseplane,px,py,pz,couter)
% NAME:ParallismText.m
% DESC:
%      TEST THE PARALLISM BETWEEN TWO PLANE FEATURES.
%------Step1:get the base-plane feature(its parameter)
%------Step2:estimate the selected plane(its compatible points to be tested)
%------Step3:evaluate the parallism
%%%%%%%%%%%%Step1%%%%%%%%%%%%
a_base=baseplane(1);
b_base=baseplane(2);
c_base=baseplane(3);
d_base=baseplane(4);
normal=baseplane(5);

%%%%%%%%%%%Step2%%%%%%%%%%%%
% number of points
N = couter;
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
options.min_iters = couter;
options.fix_seed = false;
options.reestimate = true;
options.stabilize = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RXp=zeros(3,couter);
RXp(1,:)=px;
RXp(2,:)=py;
RXp(3,:)=pz;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run RANSAC
[results1, options] = RANSAC(RXp, options);
res=results1;
distance=zeros(1,100);
j=1;
for i=1:couter
    if (results1.CS(i)==1)
            distance(1,j)=(a_base*px(i)+b_base*py(i)+c_base*pz(i)+d_base)/normal;
            j=j+1;
    end
end
    parallelvalue=max(distance)-min(distance);
    %disp(results1.CS)
    %disp(parallelvalue);
return