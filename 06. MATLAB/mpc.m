% MPC
clc
clear all
close all

% ss
r = 0.035;
T = 0.03;
d = 0.011;

vref = r / 4 * 20;
thetaz_ref = 0;

A = [% dx dy dtheta                dw12                dw34
       0  0  -vref*sin(thetaz_ref) r/2*cos(thetaz_ref) r/2*cos(thetaz_ref);
       0  0  -vref*cos(thetaz_ref) r/2*sin(thetaz_ref) r/2*sin(thetaz_ref);
       0  0  0                     -r/2/d              r/2/d              ;
       0  0  0                     -1/T                0                  ;
       0  0  0                     0                   -1/T               ;
    ]

B = [% w12 w34
       0   0   ;
       0   0   ;
       0   0   ;
       1/T 0   ;
       0   1/T ;
    ]

Q = diag([10 10 10 100 100])

R = diag([100 100])

K = lqr(A, B, Q, R)