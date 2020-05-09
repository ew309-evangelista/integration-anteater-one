%% SCRIPT_EW309corona_FinalProject_Skeleton
% 
%   M. Kutzer, 14Apr2020, USNA
clear all
close all
clc

%% (0) Create status figure
fig = figure('Name','SCRIPT_EW309corona_FinalProject_Skeleton');
axs = axes('Parent',fig);

%% (1.1) Define target information
targetRange = 350;              % STUDENT DEFINED VALUE IN CENTIMETERS
targetDiameter = 12.7;          % STUDENT DEFINED VALUE IN CENTIMETERS
targetHorizontalBias = 0.00032*targetRange + 3.2635;    % this equation was taken from the ballistics data given to us and then adjusted slightly
targetVerticalBias = 0.0000422*targetRange - 30.3554;       % same as x_bias
targetSpecs = createTargetSpecs(targetDiameter,targetHorizontalBias,targetVerticalBias); %function in the computer vision folder of the corona upload.

%% (1.2) Define test range and probability of a hit
pHit = 0.95;                    % Instructor defined value
S_P = (0.009*targetRange + 0.7983); %precision error based on ballistics data given for different ranges
CEP = targetDiameter; %sircular error probable based on the size of the circle. Making this value smaller would cause more shots.
k = CEP/S_P; %taken from powerpoint
pone = 1 - exp(-0.5*(k^2)); %stats
n = (log(1-pHit))/(log(1-pone)); %the number of shots taken

%% (1.3) Get image of target at random angle between -25deg & 25deg
im = getTargetImage(targetRange,[],targetSpecs);
img = imshow(im,'Parent',axs);
hold(axs,'on');
set(axs,'Visible','on');
drawnow
saveas(fig,'InitialImage.png','png');

%% (2) Process the image, range, target diameter, and probability of a hit 
%      to get desired angle and number of shots. 

theta_desired = deg2rad(10);   % I believe the degree is given at the start between -25 and 25.
nShots = ceil(n);                 % rounds the n value to the next whole number

%% (5) Define control parameters
K = 4.6655            % All control parameters were solve for using 5% OS and 1s ts from the other script called EW305LEAD and the open loop tf
zlead = 2.4469
p = 10.6238
Kp = (K*(zlead+zlead)*p - (K*zlead*zlead))/(p^2);
Ki = (K*zlead*zlead)/p;
Kd = (K*((p^2) - (zlead+zlead))*p + (zlead*zlead))/(p^3)

cParams.Kp = Kp;               % STUDENT DEFINED VALUE
cParams.Ki = Ki;               % STUDENT DEFINED VALUE
cParams.Kd = Kd;               % STUDENT DEFINED VALUE
cParams.despos = theta_desired;

%% (6) Define stop condition (e.g. evaluation time)
tf = 20;                         % STUDENT DEFINED VALUE IN SECONDS
t = linspace(0,tf,50);

%% (7) Run the turret model
[SSE,ts,tOUT,theta,omega,duty_cycle,eint] = sendCmdtoDcMotor('closed',cParams,t);

%% (8) Print steady state error
fprintf('SSE = %.7f\n',SSE); %our steady state error is within 1 degree.

%% (9) Get image of target after turret is moved
im = getTargetImageUpdate(theta(end));
set(img,'CData',im);
drawnow;
saveas(fig,'UpdatedImage.png','png');

%% (10) Repeat steps 2 - 9 as desired by student

%% (11) Fire at target
im = getShotPatternImage(nShots);
set(img,'CData',im);
drawnow;
saveas(fig,'ShotsImage.png','png');

%% (12) Performance assesment
EW309coronaPerforanceEval;
saveas(gcf,'ShotPerformance.png','png');