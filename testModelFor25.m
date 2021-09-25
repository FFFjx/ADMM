clc;clear;
%parameters
lambda1 = 1; % It is just used to remind setting lambda1 in line 34.
lambda2 = 1; % It is just used to remind setting lambda2 in line 35.
T = 10; % Time.
R = 4; % Freshness.
W = 100; % Car capacity.
c = 25; % Customers.
s = c+2; % Space node.
alpha2 = 2; % Time window penalty for delivery late.
rho = 0.5; % Rho.
bestK = 100; % It's actually useful when c>100. Mainly used in FDP.
iteration = 50; % ADMM iterations.

% Read data.
node = xlsread('inputFor25.xlsx', 1, 'B2:E26', 'basic'); 
demand = node(:, 1);
%et = node(:, 2);
lt = node(:, 3);
freshness = node(:, 4);
K = ceil(sum(demand)/W)+2; % Set vehicle number. It can be changed to other formula.
clear node;
link = xlsread('inputFor25.xlsx', 2, 'D2:D730', 'basic');
link = reshape(link, [s, s]);
[ UB1 ] = CalculateObject1UB( link );
link = reshape(link, [s, s, 1, 1]);

C = zeros(s, s, T+1, T+1); % Arc cost matrix. Vehicles start from t=0.
for t1 = 1:T+1
    for t2 = t1+1:T+1
        C(:, :, t1, t2) = link;
    end
end
lambda1 = 1*UB1; % Set the actual lambda1.
lambda2 = 1*UB1; % Set the actual lambda2.
k1 = lambda1/(UB1-0);
[ UB2 ] = CalculateObject2UB( demand, lt, T, alpha2 );
k2 = lambda2/(UB2-0);

% Generalized the acr cost matrix. Add the late penalty in it.
[ GC ] = GeneralizedCostMatrix( C, alpha2, lt, demand, k1, k2 );

% ADMM
[ finalRout, LocalUB, LocalLB, unServedNode, serviceTimes, visitedSeq ] = ...
    ADMM( C, GC, demand, bestK, W, K, c, iteration, rho );
% plot
x = linspace(1, iteration, iteration);
y1 = LocalUB;
y2 = LocalLB;
subplot(1, 2, 1)
plot(x, y1, x, y2);
xlabel('Iteration')
ylabel('UB & LB')
legend('UB','LB')

subplot(1, 2, 2)
plot(x, unServedNode);
xlabel('Iteration')
ylabel('The number of unserved Node')

% Calaulate primal cost for each vehicle in iteration 50.
it50Seq = squeeze(visitedSeq(:, :, 50));
[ it50 ] = CalculateCostForVehicle( it50Seq, C, GC );

% Suggest to use this function after program has been converged.
[ finalSeq, timeSeq ] = FeasibleSolution( visitedSeq, finalRout, serviceTimes, C );

% Generate freshness deviation and freshness choice for each vehicle.
[ proposalR_K, freshnessDeviation ] = ...
    GenerateFreshness( finalSeq, timeSeq, freshness, R );
