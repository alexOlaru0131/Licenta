%%
clc
clear all
close all
%

%%
motor = readtable("motor.csv");

t = motor.time;

%%
clc

M1 = motor.M1 / 4;
M1_copy = M1;
M1_10 = M1;
M1_100 = M1;

input_M1 = zeros(length(M1_copy), 1);
input_M1(201:300) = 12 * 0.2;
input_M1(301:400) = 12 * 0.4;
input_M1(401:500) = 12 * 0.6;
input_M1(501:600) = 12 * 0.8;
input_M1(601:700) = 12;
input_M1(1401:1500) = 12 * 0.2;
input_M1(1501:1600) = 12 * 0.4;
input_M1(1601:1700) = 12 * 0.6;
input_M1(1701:1800) = 12 * 0.8;
input_M1(1801:1900) = 12;
input_M1(2001:2100) = 12 * 0.4;
input_M1(2101:2200) = 12 * 0.8;
input_M1(2501:2600) = 12 * 0.4;
input_M1(2601:2700) = 12 * 0.8;

for i = 10:10:length(M1_10)
    if i <= 10
        M1_10(1 : 10) = max(M1_copy(1 : 10));
        i = i + 1;
    else
        if input_M1(i - 10 + 1 : i - 1) == 0
            M1_10(i - 10 : i) = 0;
        elseif mean(M1_copy(i - 10 : i)) > 30
            M1_100(i - 10 : i) = 0;
        else
            M1_10(i - 10 : i) = max(M1_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M1_100)
    if i <= 100
        M1_100(1 : 100) = mean(M1_10(1 : 100));
        i = i + 1;
    else
        if input_M1(i - 100 + 1 : i - 1) == 0
            M1_100(i - 100 : i) = 0;
        elseif mean(M1_copy(i - 100 : i)) > 30
            M1_100(i - 100 : i) = 0;
        else
            M1_100(i - 100 : i) = mean(M1_10(i - 100 : i));
        end
    end
end

subplot(221), plot(t, [M1_copy M1_10 M1_100 input_M1]), ylim([0 40]), xlim([0 2700])
title("Motor 1"), legend(["Raw", "Rotations [1s]", "Rotations [10s]" "Input signal"])

%
M2 = motor.M2 / 4;
M2_copy = M2;
M2_10 = M2;
M2_100 = M2;

input_M2 = zeros(length(M2_copy), 1);
input_M2(201:300) = 12 * 0.2;
input_M2(301:400) = 12 * 0.4;
input_M2(401:500) = 12 * 0.6;
input_M2(501:600) = 12 * 0.8;
input_M2(601:700) = 12;
input_M2(1401:1500) = 12 * 0.2;
input_M2(1501:1600) = 12 * 0.4;
input_M2(1601:1700) = 12 * 0.6;
input_M2(1701:1800) = 12 * 0.8;
input_M2(1801:1900) = 12;
input_M2(2001:2100) = 12 * 0.4;
input_M2(2101:2200) = 12 * 0.8;
input_M2(2501:2600) = 12 * 0.4;
input_M2(2601:2700) = 12 * 0.8;

for i = 10:10:length(M2_10)
    if i <= 10
        M2_10(1 : 10) = max(M2_copy(1 : 10));
        i = i + 1;
    else
        if input_M2(i - 10 + 1 : i - 1) == 0
            M2_10(i - 10 : i) = 0;
        elseif mean(M2_copy(i - 10 : i)) > 30
            M2_100(i - 10 : i) = 0;
        else
            M2_10(i - 10 : i) = max(M2_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M2_100)
    if i <= 100
        M2_100(1 : 100) = mean(M2_10(1 : 100));
        i = i + 1;
    else
        if input_M2(i - 100 + 1 : i - 1) == 0
            M2_100(i - 100 : i) = 0;
        elseif mean(M2_copy(i - 100 : i)) > 30
            M2_100(i - 100 : i) = 0;
        else
            M2_100(i - 100 : i) = mean(M2_10(i - 100 : i));
        end
    end
end

subplot(222), plot(t, [M2_copy M2_10 M2_100 input_M2]), ylim([0 40]), xlim([0 2700])
title("Motor 2"), legend(["Raw", "Rotations [1s]", "Rotations [10s]" "Input signal"])

%
M3 = motor.M3 / 4;
M3_copy = M3;
M3_10 = M3;
M3_100 = M3;

input_M3 = zeros(length(M3_copy), 1);
input_M3(201:300) = 12 * 0.2;
input_M3(301:400) = 12 * 0.4;
input_M3(401:500) = 12 * 0.6;
input_M3(501:600) = 12 * 0.8;
input_M3(601:700) = 12;
input_M3(801:900) = 12 * 0.2;
input_M3(901:1000) = 12 * 0.4;
input_M3(1001:1100) = 12 * 0.6;
input_M3(1101:1200) = 12 * 0.8;
input_M3(1201:1300) = 12;
input_M3(2001:2100) = 12 * 0.4;
input_M3(2101:2200) = 12 * 0.8;
input_M3(2301:2400) = 12 * 0.4;
input_M3(2401:2500) = 12 * 0.8;

for i = 10:10:length(M3_10)
    if i <= 10
        M3_10(1 : 10) = max(M3_copy(1 : 10));
        i = i + 1;
    else
        if input_M3(i - 10 + 1 : i - 1) == 0
            M3_10(i - 10 : i) = 0;
        elseif mean(M3_copy(i - 10 : i)) > 30
            M3_100(i - 10 : i) = 0;
        else
            M3_10(i - 10 : i) = max(M3_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M3_100)
    if i <= 100
        M3_100(1 : 100) = mean(M3_10(1 : 100));
        i = i + 1;
    else
        if input_M3(i - 100 + 1 : i - 1) == 0
            M3_100(i - 100 : i) = 0;
        elseif mean(M3_copy(i - 100 : i)) > 30
            M3_100(i - 100 : i) = 0;
        else
            M3_100(i - 100 : i) = mean(M3_10(i - 100 : i));
        end
    end
end

subplot(223), plot(t, [M3_copy M3_10 M3_100 input_M3]), ylim([0 40]), xlim([0 2700])
title("Motor 3"), legend(["Raw", "Rotations [1s]", "Rotations [10s]" "Input signal"])

%
M4 = motor.M4 / 4;
M4_copy = M4;
M4_10 = M4;
M4_100 = M4;

input_M4 = zeros(length(M4_copy), 1);
input_M4(201:300) = 12 * 0.2;
input_M4(301:400) = 12 * 0.4;
input_M4(401:500) = 12 * 0.6;
input_M4(501:600) = 12 * 0.8;
input_M4(601:700) = 12;
input_M4(801:900) = 12 * 0.2;
input_M4(901:1000) = 12 * 0.4;
input_M4(1001:1100) = 12 * 0.6;
input_M4(1101:1200) = 12 * 0.8;
input_M4(1201:1300) = 12;
input_M4(2001:2100) = 12 * 0.4;
input_M4(2101:2200) = 12 * 0.8;
input_M4(2301:2400) = 12 * 0.4;
input_M4(2401:2500) = 12 * 0.8;

for i = 10:10:length(M4_10)
    if i <= 10
        M4_10(1 : 10) = max(M4_copy(1 : 10));
        i = i + 1;
    else
        if input_M4(i - 10 + 1 : i - 1) == 0
            M4_10(i - 10 : i) = 0;
        elseif mean(M4_copy(i - 10 : i)) > 30
            M4_100(i - 10 : i) = 0;
        else
            M4_10(i - 10 : i) = max(M4_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M4_100)
    if i <= 100
        M4_100(1 : 100) = mean(M4_10(1 : 100));
        i = i + 1;
    else
        if input_M4(i - 100 + 1 : i - 1) == 0
            M4_100(i - 100 : i) = 0;
        elseif mean(M4_copy(i - 100 : i)) > 30
            M4_100(i - 100 : i) = 0;
        else
            M4_100(i - 100 : i) = mean(M4_10(i - 100 : i));
        end
    end
end

subplot(224), plot(t, [M4_copy M4_10 M4_100 input_M4]), ylim([0 40]), xlim([0 2700])
title("Motor 4"), legend(["Raw", "Rotations [1s]", "Rotations [10s]" "Input signal"])

%%
figure, plot(t, [M1_10 M1_100 input_M1]), ylim([0 30]), xlim([0 1900])
title("Motor 1"), legend(["Rotations [1s]", "Rotations [10s]" "Input signal"])

%%
figure, plot(t, [M2_10 M2_100 input_M2]), ylim([0 30]), xlim([0 1900])
title("Motor 2"), legend(["Rotations [1s]", "Rotations [10s]" "Input signal"])

%%
figure, plot(t, [M3_10 M3_100 input_M3]), ylim([0 30]), xlim([0 1900])
title("Motor 3"), legend(["Rotations [1s]", "Rotations [10s]" "Input signal"])

%%
figure, plot(t, [M4_10 M4_100 input_M4]), ylim([0 30]), xlim([0 1900])
title("Motor 4"), legend(["Rotations [1s]", "Rotations [10s]" "Input signal"])

%%
close all
clc

figure, hold on, ylim([0 30]), xlim([0 2700])
M_10 = zeros(size(M1_10));
M_100 = zeros(size(M1_10));

for i = 1:1:800
    M_10(i) = mean([M1_10(i), M2_10(i), M3_10(i), M4_10(i)]);
end

for i = 901:1:1300
    M_10(i) = mean([M3_10(i), M4_10(i)]);
end

for i = 1601:1:1900
    M_10(i) = mean([M1_10(i), M2_10(i)]);
end

M_100(1800:1900) = mean(M_10(1800:1900));
for i = 1:100:1800
    M_100(i:i+100) = mean(M_10(i:i+100));
end

input = zeros(size(input_M1));
for i = 1:1:1900
    if input_M1(i) == 0 & input_M3(i) ~= 0
        input(i) = input_M3(i);
    elseif input_M1(i) ~= 0 & input_M3(i) == 0
        input(i) = input_M1(i);
    elseif input_M1(i) ~= 0 & input_M3(i) ~= 0
        input(i) = input_M3(i);
    end
end

plot(t, [M1_10 M2_10 M3_10 M4_10])
plot(t, M_10, 'LineWidth', 1)
plot(t, M_100, 'LineWidth', 2)
plot(t, input)
legend(["M1 Rotations [1s]" "M2 Rotations [1s]" "M3 Rotations [1s]" "M4 Rotations [1s]" "Mean rotations [1s]" "Mean rotations [10s]" "Input signal"])

%% Regulatoare M1/M2
u0 = 0;
ust = 1;

y0 = 0;
yst = 24;

Kp = (yst - y0)/(ust - u0);

y63 = y0 + 0.63*(yst - y0);

Tp = 0.13;
Hp= tf(Kp, [Tp 1]);
Hpd = c2d(Hp, 1, "zoh");

Ho = tf(1, [Tp 1]);
Hod = c2d(Ho, 1, "zoh");

Hrd = minreal(Hod / Hpd / (1 - Hod))

%