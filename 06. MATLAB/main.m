%%
clc
close all
%

figure
t = motor(:,5);

M1 = motor(:,1);
M1_copy = M1;

for i = 4:1:length(M1)-2
    sum = 0;
    sum = sum + M1(i-2) + M1(i-1) + M1(i) + M1(i+1) + M1(i+2);
    M1(i) = floor(sum/5);
end

M1_step = 5;
M1_movemean = zeros(length(M1),1);
for j = M1_step:5:30
    for i = j+1:j:length(M1)
        M1_movemean(i-j:i,1) = movmean(M1_copy(i-j:i, 1), j);
    end
end

subplot(221), plot(t, [M1 M1_copy]), ylim([0 150])

M2 = motor(:,2);
M2_copy = M2;

for i = 4:1:length(M2)-2
    sum = 0;
    sum = sum + M2(i-2) + M2(i-1) + M2(i) + M2(i+1) + M2(i+2);
    M2(i) = floor(sum/5);
end

M2_step = 5;
M2_movemean = zeros(length(M2),1);
for j = M2_step:5:30
    for i = j+1:j:length(M2)
        M2_movemean(i-j:i,1) = movmean(M2_copy(i-j:i, 1), j);
    end
end

subplot(222), plot(t, [M2 M2_copy]), ylim([0 150])

M3 = motor(:,3);
M3_copy = M3;

for i = 4:1:length(M3)-2
    sum = 0;
    sum = sum + M3(i-2) + M3(i-1) + M3(i) + M3(i+1) + M3(i+2);
    M3(i) = floor(sum/5);
end

M3_step = 5;
M3_movemean = zeros(length(M3),1);
for j = M3_step:5:30
    for i = j+1:j:length(M3)
        M3_movemean(i-j:i,1) = movmean(M3_copy(i-j:i, 1), j);
    end
end

subplot(223), plot(t, [M3 M3_copy]), ylim([0 150])

M4 = motor(:,4);
M4_copy = M4;

for i = 4:1:length(M4)-2
    sum = 0;
    sum = sum + M4(i-2) + M4(i-1) + M4(i) + M4(i+1) + M4(i+2);
    M4(i) = floor(sum/5);
end

M4_step = 5;
M4_movemean = zeros(length(M4),1);
for j = M4_step:5:30
    for i = j+1:j:length(M4)
        M4_movemean(i-j:i,1) = movmean(M4_copy(i-j:i, 1), j);
    end
end

subplot(224), plot(t, [M4 M4_copy]), ylim([0 150])


%