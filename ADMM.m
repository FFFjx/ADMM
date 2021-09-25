function [ finalRout, LocalUB, LocalLB, unServedNode, serviceTimes, visitedSeq ] = ...
    ADMM( C, GC, demand, bestK, W, K, c, iteration, rho )
% ADMM can be used to optimize a K-vehicles VRP problem from an infeasible
% solution to a feasible solution through iterations. It decomposes the
% whole large-scale problem into K independent VRP which can be easily
% solved by using K FDP.
%% Initialize the variables that is used in the function or for output
    s = c+2;
    stepsize = rho;
    LocalUB = zeros(1, iteration); % Record UB in each iteration.
    LocalLB = zeros(1, iteration); % Record LB in each iteration.
    
    % Record the service times for each node in each iteration.
    serviceTimes = zeros(iteration, s); 
    
    % Record the service status for each vehicle in each iteration. The
    % (s+1)th column labels that whether this vehicle has been routing or
    % not.
    finalRout = zeros(K, s+1, iteration);
    
    % Record the visited sequence for each vehicle in each iteration. s1=1
    % stands for the collective begining node, and ss=s stands for the
    % collective ending node.
    visitedSeq = zeros(K, s, iteration);
    
    % Record the number of unserved nodes in each iteration.
    unServedNode = zeros(1, iteration); 
    
    profit = zeros(2, c); % Extra profit for ADMM(row 1) and LR(row 2).
    
    % Record LR profit for each customers in each iteration.
    recordProfit = zeros(iteration, c);
   %% Run the iteration.
    for it=1:iteration
        disp('iteration=');
        disp(it);
        usedV = 0;
        
        % Copy the routing status and service times from last iteration.
        if it~=1
            serviceTimes(it, :) = serviceTimes(it-1, :);
            finalRout(:, :, it) = finalRout(:, :, it-1);
        end
       %% Calculate upper bound(it).
        
        for k=1:K
            % If the vehicle has been routing from last iteration, cancel
            % this routing and reset the service times for this single
            % routing. Ensure the next rerouting is based on the fixed
            % routings from any other vehicle.
            if finalRout(k, s+1, it)~=0
                for n=1:s
                    serviceTimes(it, n) = serviceTimes(it, n)-...
                        finalRout(k, n, it);
                end
            end
            
            % Update the profit part by equation (20)
            for n=1:s-2
                profit(1, n) = profit(2, n)+...
                    (1-2*serviceTimes(it, n+1))*rho/2;
            end

            % FDP
            [ completeRout ] = ...
                FDP( C, GC, profit, demand, bestK, W, 0 );
            
            % Sort every complete routing we can get from FDP by ADMM cost.
            completeRout = sortrows(completeRout, s+2);
            
            % Record the smallest ADMM cost sequence. 
            visitedSeq(k, :, it) = completeRout(1, 1:s);
            
            % To calculate upper bound(it) using primal cost.
            LocalUB(1, it) = LocalUB(1, it)+completeRout(1,s+1);
            
            % Reset service status and record the new one.
            finalRout(k, :, it) = 0;
            for n=1:s
                cn = completeRout(1, n);
                if cn~=Inf
                    finalRout(k, cn, it) = 1;
                    serviceTimes(it, cn) = serviceTimes(it, cn)+1;
                end
            end
            finalRout(k, s+1, it) = 1;
            
            % Calaulate the actual used vehicles number.
            if sum(completeRout(1, 1:s)~=Inf)~=2
                usedV = usedV+1;
            end
        end
        
        % Calculate the number of unserved nodes and based it to add a
        % penalty to UB.
        unServed = sum(serviceTimes(it, :)==0);
        LocalUB(1, it) = LocalUB(1, it)+unServed*100;
       %% Calculate lower bound(it).

        % Run a single FDP.
        [ completeRoutLR ] = ...
            FDP( C, GC, profit, demand, bestK, W, 1 );
        completeRoutLR = sortrows(completeRoutLR, s+3);
        for n=1:s-1
            if completeRoutLR(n, s+1)==Inf
                routNumLR = n-1;
                break;
            end
        end
        
        % Get (usedV) shortest paths and use it to calaulate LB.
        for vv=1:min(usedV, routNumLR)
            LocalLB(1, it) = LocalLB(1, it)+completeRoutLR(vv, s+3);
        end
        for n=1:s-2
            LocalLB(1, it) = LocalLB(1, it)+profit(2, n);
        end
        
        % Record the number of unserved nodes in each iteration.
        unServedNode(1, it) = unServed;
       %% Update the quadratic penalty.
        if it>=20 % This 20 can be changed. 
            repeatServed = sum(serviceTimes(it, :)>1);
            lastUnServed = sum(serviceTimes(it-1, :)==0);
            lastRepeatServed = sum(serviceTimes(it-1, :)>1);
            s1 = (unServed+repeatServed)*(unServed+repeatServed);
            s0 = (lastUnServed+lastRepeatServed)*...
                (lastUnServed+lastRepeatServed);
            if s1>0.25*s0
                rho = rho+stepsize;
            end
            if s1==0
                rho=0;
            end
        end      
       %% Update the extra profit for LR.
        for n=1:s-2
            recordProfit(it, n) = profit(2, n);
            profit(2, n) = profit(2, n)+(1-serviceTimes(it, n+1))*rho;
        end
    end

end

