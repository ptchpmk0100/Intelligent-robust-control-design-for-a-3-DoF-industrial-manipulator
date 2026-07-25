function [sys,x0,str,ts] = NTSMCinput(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;  % qd1, dqd1, ddqd1, qd2, dqd2, ddqd2, qd3, dqd3, ddqd3
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

%---------------------------------------Output---------------------------------------%
function sys=mdlOutputs(t,x,u)
%-----------------CONST WAVE-----------------%
% sys(1) = 1;
% sys(2) = 0;
% sys(3) = 0;
% sys(4) = 1.5;
% sys(5) = 0;
% sys(6) = 0;
% sys(7) = 2;
% sys(8) = 0;
% sys(9) = 0;

%-----------------SINE WAVE------------------%
% for AC motors
% qd1=0.1*sin(t);
% dqd1=0.1*cos(t);
% ddqd1=-0.1*sin(t);
% qd2=0.1*sin(t);
% dqd2=0.1*cos(t);
% ddqd2=-0.1*sin(t);
% qd3=0.1*sin(t);
% dqd3=0.1*cos(t);
% ddqd3=-0.1*sin(t);

% qd1=0.5*sin(t);
% dqd1=0.5*cos(t);
% ddqd1=-0.5*sin(t);
% qd2=0.5*sin(t);
% dqd2=0.5*cos(t);
% ddqd2=-0.5*sin(t);
% qd3=0.5*sin(t);
% dqd3=0.5*cos(t);
% ddqd3=-0.5*sin(t);
% 
% sys(1)=qd1;
% sys(2)=dqd1;
% sys(3)=ddqd1;
% sys(4)=qd2;
% sys(5)=dqd2;
% sys(6)=ddqd2;
% sys(7)=qd3;
% sys(8)=dqd3;
% sys(9)=ddqd3;

% -----------------SQUARE WAVE------------------%
% for stepper motors
% Generating a square wave with period T=2*pi, amplitude = 0.1
% qd1 = 0.1*square(t);   
% dqd1 = 0;              
% ddqd1 = 0;              
% 
% qd2 = 0.1*square(t);   
% dqd2 = 0;               
% ddqd2 = 0;              
% 
% qd3 = 0.1*square(t);   
% dqd3 = 0;                
% ddqd3 = 0;               

qd1 = 0.5*square(t);     
dqd1 = 0;                
ddqd1 = 0;  
qd2 = 0.5*square(t);     
dqd2 = 0;                
ddqd2 = 0;  
qd3 = 0.5*square(t);     
dqd3 = 0;                
ddqd3 = 0;  

% Store values in the sys array
sys(1) = qd1;
sys(2) = dqd1;
sys(3) = ddqd1;
sys(4) = qd2;
sys(5) = dqd2;
sys(6) = ddqd2;
sys(7) = qd3;
sys(8) = dqd3;
sys(9) = ddqd3;
