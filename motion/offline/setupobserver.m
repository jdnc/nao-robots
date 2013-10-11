% for robot
dt = input("dt:")
numPreviewFrames = input("numPreviewFrames:")
%dt = 0.01;
%numPreviewFrames = 100;
% for sim
% dt = 0.02;
%numPreviewFrames = 45;

%z_h = 240; %270; %310; %360; %235; %250.456;
z_h = input("z_h:")

R = 1*10^(-10);
Qx = 0.4;  Qe = 0.3;
Ql = [1,0,0;0,1,0;0,0,1];

RO = [100000,  0      ;
            0,  100000]; 


g = 9806.65;

A0 = [1, dt, 0; g/z_h*dt, 1, -g/z_h*dt; 0, 0, 1];
b0 = [0; 0; dt];
c0 = [0, 0, 1];

Co=ctrb(A0, b0);
unco=length(A0)-rank(Co)


Bt(1,1)=c0*b0;
Bt(2:4,1)=b0(1:3);
It(1,1)=1;
It(2:4,1)=0;
Ft(1,1:3)=c0*A0;
Ft(2:4,1:3)=A0(1:3,1:3);
Qt(1:4, 1:4)=0;
Qt(1,1)=Qe;
Qt(2:4,2:4)=c0' *Qx*c0;
At(1:4,1)=It;
At(1:4,2:4)=Ft;

Co=ctrb(At, Bt);
unco=length(At)-rank(Co)

%[Pt, L, G, report]=dare(At, Bt, Qt, R);
Pt = dare(At, Bt, Qt, R);
%report

Gx = (1/(R+Bt'*Pt*Bt)) * Bt'*Pt*Ft;
Gi = (1/(R+Bt'*Pt*Bt)) * Bt'*Pt*It;

Ac = At - Bt*(1/(R + Bt'*Pt*Bt)) * Bt'*Pt*At;
X = -Ac'*Pt*It;
Gd(1) = -Gi;
for i=2:numPreviewFrames,
  Gd(i) = (1/(R + Bt'*Pt*Bt))*Bt'*X;
  X = Ac'*X;
end

Cm=[1,0,0;
    0,0,1];

Ob=obsv(A0, Cm);
ObserverRank=rank(Ob)
L = dlqr(A0.', Cm.', Ql , RO)'



%A = A0-b0*Gx;

%L = dlqr(A', c0', Ql , R)';

endTime = 10;
for time=[0:dt:endTime],
  preview_frames = [];

  for j=[1:numPreviewFrames],
    preview_frames(j) = real( Gd(j)*(time + j*dt) );
  end;
end;

% save results
save -6 results.mat Gi Gx Gd A0 b0 L c0
