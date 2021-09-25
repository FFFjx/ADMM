clc;clear;
%parameters
lambda1 = 1;
lambda2 = 1;
T = 10; % Time.
R = 4; % Freshness.
W = 100; % Car capacity.
c = 100; % Customers.
s = c+2; % Space node.
alpha2 = 2; % Time window penalty for delivery late.
rho = 0.1;
bestK = 100;
iteration = 100; % ADMM iterations.

% The next 3 lines of code is used to generate random time intervals
% between random 2 nodes. Writes it in an excel file and then copy it
% to 'inputFor100.xlsx' file.
%timeRange = [1, 3];
%[ B ]  =  GenerateRandomTime( c, timeRange );
%xlswrite('randomTimeFor100', B);

node = xlsread('inputFor100.xlsx', 1, 'B2:E101', 'basic');
demand = node(:, 1);
%et = node(:, 2);
lt = node(:, 3);
freshness = node(:, 4);
K = ceil(sum(demand)/W)+2; % Set vehicle number.
clear node;
link = xlsread('inputFor100.xlsx', 2, 'D2:D10405', 'basic');

link = reshape(link, [s, s]);
[ UB1 ] = CalculateObject1UB( link );
link = reshape(link, [s, s, 1, 1]);
C = zeros(s, s, T+1, T+1); % Arc cost matrix. Vehicles start from t=0.
for t1 = 1:T+1
    for t2 = t1+1:T+1
        C(:, :, t1, t2) = link;
    end
end
lambda1 = 1*UB1;
lambda2 = 1*UB1;
k1 = lambda1/(UB1-0);
[ UB2 ] = CalculateObject2UB( demand, lt, T, alpha2 );
k2 = lambda2/(UB2-0);
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

% Suggest to use this function after program has been converged.
[ finalSeq, timeSeq ] = FeasibleSolution( visitedSeq, finalRout, serviceTimes, C );

% Generate freshness deviation and freshness choice for each vehicle.
[ proposalR_K, freshnessDeviation ] = ...
    GenerateFreshness( finalSeq, timeSeq, freshness, R );
