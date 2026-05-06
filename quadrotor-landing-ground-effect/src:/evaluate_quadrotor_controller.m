% Number of trials
n = 100;

% Landing target
z0 = 0.001; 
tol = 0.1 * z0;  % 10% tolerance

% Results storage
successes = 0;
settleTimes = [];

for i = 1:n
    fprintf('Running simulation %d of %d \n',i,n)

    % Run simulation without graphics and save data
    Project1('ControllerSun','datafile','data.mat','display',false);

    % Load data
    load('data.mat');   % loads processdata, controllerdata

    % Extract time and height
    t = controllerdata.sensors.t;
    z = controllerdata.sensors.z;

    % Success check: if simulation reached full time (no crash)
    if t(end) >= 10
        successes = successes + 1;

        % Settling time: first time z within ±10% of z0
        idx = find(abs(z - z0) <= tol, 1);
        if ~isempty(idx)
            settleTimes(end+1) = t(idx);
        end
    end
end

% Compute metrics
successRate = successes / n * 100;
if ~isempty(settleTimes)
    avgSettleTime = mean(settleTimes);
else
    avgSettleTime = NaN;
end

% Display results
fprintf('\n==== Evaluation Results ====\n');
fprintf('Success rate: %.1f%% (%d/%d)\n', successRate, successes, n);
fprintf('Average settling time: %.3f s\n', avgSettleTime);