% Note: 0h09' 05-11-2024
% Goal: Use the average value of e and de for clustering => Get 7 cluster
% Change:
    % Change e and de calculation
    % Change reference signal in controller input.m and plant.m from 0.1*sint(t) -> 1*sin(t)
    % Change min dist for merging in step 6 of MNNC.m to 0.20 
    % Optimal: choose first 2000-3000 samples

H = 5; Nmin = 5;

% e = zeros(1000,1); de = zeros(1000,1);
e = zeros(2000,1); de = zeros(2000,1);
% e = zeros(3000,1); de = zeros(3000,1);
% e = zeros(5000,1); de = zeros(5000,1);

for i = 1:length(e)
    e(i) = (1/3)*(e1(i) + e2(i) + e3(i));
    de(i) = (1/3)*(de1(i) + de2(i) + de3(i));
end

data = [e';de']; size(data)

clusters = MNNC(data, H, Nmin);