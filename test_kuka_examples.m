
% Snapshots of real measurements of joint angle and cartesian position,
% taken from the physical robot.

joints = [
    -0.01 0.01 0.01 0.00 -0.01 0.01 0.00;         % 1; very near singular
    -0.01 -35.10 0.00 0.00 -0.01 0.01 0.00;
    -0.01 -35.10 47.58 0.00 0.00 0.00 0.00;
    -0.01 -35.10 47.58 24.17 0.00 0.00 0.00;
    -0.01 -35.10 47.58 24.17 -34.71 0.00 0.00;
    -0.01 -35.10 47.58 24.17 -34.71 33.49 0.00;
    -0.01 -35.10 47.58 24.17 -34.71 33.49 31.17;
    -0.01 0.00 0.00 0.00 0.01 0.00 0.00;          % 8; singular
    -0.01 0.00 0.00 0.00 0.00 0.00 36.00;         % 9; singular
    -0.01 0.00 0.00 0.00 0.00 -46.35 36.10;       % 10; singular
    -0.01 0.00 0.00 0.00 36.14 -46.52 36.10;      % 11; singular
    -0.01 0.00 0.00 -26.56 36.14 -46.52 36.10;    % 12
    -0.01 0.00 -41.51 -26.62 36.14 -46.52 36.10;  % 13
    -0.01 19.56 -41.60 -26.62 36.14 -46.52 36.10; % 14
    24.12 19.56 -41.59 -26.62 36.14 -46.52 36.10; % 15
    0.06 -33.58 0.00 0.01 0.00 0.00 0.00;
    ];
cart = [  % x,y,z,A,B,C    A,B,C are euler angles ZYX in degrees
    0.13 0 1306 0 0.01 0;
    -543.92 0.05 1133.99 0 -35.09 0;
    -543.92 0.06 1134.01 53.21 -22.82 -27.42;
    -636.26 -158.90 1012.77 68.28 -43.54 -35.83;
    -636.26 -158.91 1012.77 16.72 -53.89 4.21;
    -581.34 -140.80 1056.64 19.20 -20.46 2.65;
    -581.34 -140.80 1056.64 52.29 -18.75 -8.74;
    -0.02 0 1306 0 -0.01 0;
    -0.01 0 1306 36.00 0.00 0.00;
    -91.18 0.00 1266.97 46.57 -35.78 -31.71;
    -73.85 -53.91 1266.70 82.80 -35.89 -31.86;
    151.56 -53.94 1248.36 66.29 -28.62 -1.50;
    78.14 -141.18 1248.19 24.75 -28.60 -1.43;
    370.83 -141.32 1170.82 21.88 -10.59 6.74;
    396.20 22.63 1170.82 46.01 -10.59 6.74;
    -523.32 -0.59 1148.07 0.06 -33.59 0.00;
    ];
