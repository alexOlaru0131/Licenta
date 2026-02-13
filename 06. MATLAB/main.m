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
close all

M1 = motor.M1;
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
input_M1(2401:2500) = 12 * 0.4;
input_M1(2501:2600) = 12 * 0.8;
input_M1(2601:2700) = 12 * 0.4;
input_M1(2701:2800) = 12 * 0.8;

for i = 10:10:length(M1_10)
    if i <= 10
        M1_10(1 : 10) = mean(M1_copy(1 : 10));
        i = i + 1;
    else
        if input_M1(i - 10 + 1 : i - 1) == 0
            M1_10(i - 10 : i) = 0;
        elseif mean(M1_copy(i - 10 : i)) > 30
            M1_100(i - 10 : i) = 0;
        else
            M1_10(i - 10 : i) = mean(M1_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M1_100)
    if i <= 100
        M1_100(1 : 100) = mean(M1_copy(1 : 100));
        i = i + 1;
    else
        if input_M1(i - 100 + 1 : i - 1) == 0
            M1_100(i - 100 : i) = 0;
        elseif mean(M1_copy(i - 100 : i)) > 30
            M1_100(i - 100 : i) = 0;
        else
            M1_100(i - 100 : i) = mean(M1_copy(i - 100 : i));
        end
    end
end

subplot(221), hold on
stairs(t, M1_copy)
plot(t, [M1_10 M1_100 input_M1]), ylim([0 20]), xlim([0 2900])
title("Motor 1"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])

%
M2 = motor.M2;
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
input_M2(2401:2500) = 12 * 0.4;
input_M2(2501:2600) = 12 * 0.8;
input_M2(2601:2700) = 12 * 0.4;
input_M2(2701:2800) = 12 * 0.8;

for i = 10:10:length(M2_10)
    if i <= 10
        M2_10(1 : 10) = mean(M2_copy(1 : 10));
        i = i + 1;
    else
        if input_M2(i - 10 + 1 : i - 1) == 0
            M2_10(i - 10 : i) = 0;
        elseif mean(M2_copy(i - 10 : i)) > 30
            M2_100(i - 10 : i) = 0;
        else
            M2_10(i - 10 : i) = mean(M2_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M2_100)
    if i <= 100
        M2_100(1 : 100) = mean(M2_copy(1 : 100));
        i = i + 1;
    else
        if input_M2(i - 100 + 1 : i - 1) == 0
            M2_100(i - 100 : i) = 0;
        elseif mean(M2_copy(i - 100 : i)) > 30
            M2_100(i - 100 : i) = 0;
        else
            M2_100(i - 100 : i) = mean(M2_copy(i - 100 : i));
        end
    end
end

subplot(222), hold on
stairs(t, M2_copy)
plot(t, [M2_10 M2_100 input_M2]), ylim([0 20]), xlim([0 2900])
title("Motor 2"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])

%
M3 = motor.M3;
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
input_M3(2201:2300) = 12 * 0.4;
input_M3(2301:2400) = 12 * 0.8;
input_M3(2601:2700) = 12 * 0.4;
input_M3(2701:2800) = 12 * 0.8;

for i = 10:10:length(M3_10)
    if i <= 10
        M3_10(1 : 10) = mean(M3_copy(1 : 10));
        i = i + 1;
    else
        if input_M3(i - 10 + 1 : i - 1) == 0
            M3_10(i - 10 : i) = 0;
        elseif mean(M3_copy(i - 10 : i)) > 30
            M3_100(i - 10 : i) = 0;
        else
            M3_10(i - 10 : i) = mean(M3_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M3_100)
    if i <= 100
        M3_100(1 : 100) = mean(M3_copy(1 : 100));
        i = i + 1;
    else
        if input_M3(i - 100 + 1 : i - 1) == 0
            M3_100(i - 100 : i) = 0;
        elseif mean(M3_copy(i - 100 : i)) > 30
            M3_100(i - 100 : i) = 0;
        else
            M3_100(i - 100 : i) = mean(M3_copy(i - 100 : i));
        end
    end
end

subplot(223), hold on
stairs(t, M3_copy)
plot(t, [M3_10 M3_100 input_M3]), ylim([0 20]), xlim([0 2900])
title("Motor 3"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])

%
M4 = motor.M4;
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
input_M4(2201:2300) = 12 * 0.4;
input_M4(2301:2400) = 12 * 0.8;
input_M4(2601:2700) = 12 * 0.4;
input_M4(2701:2800) = 12 * 0.8;

for i = 10:10:length(M4_10)
    if i <= 10
        M4_10(1 : 10) = mean(M4_copy(1 : 10));
        i = i + 1;
    else
        if input_M4(i - 10 + 1 : i - 1) == 0
            M4_10(i - 10 : i) = 0;
        elseif mean(M4_copy(i - 10 : i)) > 30
            M4_100(i - 10 : i) = 0;
        else
            M4_10(i - 10 : i) = mean(M4_copy(i - 10 : i));
        end
    end
end

for i = 100:100:length(M4_100)
    if i <= 100
        M4_100(1 : 100) = mean(M4_copy(1 : 100));
        i = i + 1;
    else
        if input_M4(i - 100 + 1 : i - 1) == 0
            M4_100(i - 100 : i) = 0;
        elseif mean(M4_copy(i - 100 : i)) > 30
            M4_100(i - 100 : i) = 0;
        else
            M4_100(i - 100 : i) = mean(M4_copy(i - 100 : i));
        end
    end
end

subplot(224), hold on
stairs(t, M4_copy)
plot(t, [M4_10 M4_100 input_M4]), ylim([0 20]), xlim([0 2900])
title("Motor 4"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])

%%
close all
clc
figure, plot(t, [M1_10 M1_100 input_M1]), ylim([0 30]), xlim([0 1900])
title("Motor 1"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])
max(M1_100) * 1 / 42.6918

%%
close all
clc
figure, plot(t, [M2_10 M2_100 input_M2]), ylim([0 30]), xlim([0 2900])
title("Motor 2"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])
max(M2_100) * 1 / 42.6918
(max(M1_100) * 1 / 42.6918) / (max(M2_100) * 1 / 42.6918)

%%
close all
clc
figure, plot(t, [M3_10 M3_100 input_M3]), ylim([0 30]), xlim([0 2900])
title("Motor 3"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])
max(M3_100) * 1 / 42.6918

%%
close all
clc
figure, plot(t, [M4_10 M4_100 input_M4]), ylim([0 30]), xlim([0 2900])
title("Motor 4"), legend(["Raw", "Mean pulses [1s]", "Mean pulses [10s]", "Input signal"])
max(M4_100) * 1 / 42.6918
(max(M3_100) * 1 / 42.6918) / (max(M4_100) * 1 / 42.6918)

%% Regulator
clc

u0 = 0;
ust = 1;

y0 = 0;
yst = 42.69;

Kp = (yst - y0)/(ust - u0);

y63 = y0 + 0.63*(yst - y0);

Tp = 0.18;
Hp= tf(Kp, [Tp 1]);
Hpd = c2d(Hp, Tp, "zoh");

Ho = tf(1, [Tp 1]);
Hod = c2d(Ho, Tp, "zoh");

Hrd = minreal(Hod / Hpd / (1 - Hod))

%