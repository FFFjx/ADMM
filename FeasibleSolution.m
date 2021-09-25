function [ finalSeq, timeSeq ] = FeasibleSolution( visitedSeq, finalRout, serviceTimes, C )
% Generate final visited sequence and time sequence for each vehicle.
% Suggest to use this function after program has been converged. When the
% result of the program is unfeasible, make it feasible.
    iteration = size(visitedSeq, 3);
    K = size(finalRout, 1);
    s = size(finalRout, 2)-1;
    
    actualUsedK = 0;
    for k=1:K
        if sum(finalRout(k, 1:s, iteration))~=2
            actualUsedK = actualUsedK+1;
        end
    end
    
    % Output. Final visited sequence.
    finalSeq = 1./zeros(actualUsedK, s);
    currentK = 0;
    for k=1:K
        if sum(finalRout(k, 1:s, iteration))~=2
            currentK = currentK+1;
            finalSeq(currentK, :) = visitedSeq(k, :, iteration);
        end
    end
    
    % Output. Final time sequence.
    timeSeq = 1./zeros(actualUsedK, s);
    for k =1:actualUsedK
        timeSeq(k, 1) = 1;
        for n=2:s
            if finalSeq(k, n)==Inf
                break;
            end
            timeSeq(k, n) = 0;
            timeSeq(k, n) = timeSeq(k, n-1)+...
                C(finalSeq(k, n-1), finalSeq(k, n), 1, 2);
        end
    end
    % All the customers only served once.
    if serviceTimes(iteration, 2:s-1)==ones(1, s-2)
        return;
    end
    
    % Unfeasible situation. Only deal with situation that repeat service
    % times = 2.
    for n=2:s-1
        if serviceTimes(iteration, n)==2 % Node n is served repeatly.
            [k1, n1]=find(finalSeq==n);% Find index of node n. 
            
            % Designate the second vehicle for customer n. Deal with the
            % first vehicle path.
            for n2=n1(1, 1):s-1
                finalSeq(k1(1, 1), n2)=finalSeq(k1(1, 1), n2+1);
            end
        end
    end
    
    % Update actual UsedK. Deal with problem above may generate empty
    % vehicle.
    copySeq = finalSeq;
    K1 = actualUsedK;
    actualUsedK = 0;
    for k=1:K1
        if sum(copySeq(k, 1:s))~=2
            actualUsedK = actualUsedK+1;
        end
    end
    finalSeq = 1./zeros(actualUsedK, s);
    currentK = 0;
    for k=1:K1
        if sum(copySeq(k, 1:s))~=2
            currentK = currentK+1;
            finalSeq(currentK, :) = copySeq(k, :);
        end
    end
    
    % If the customer is not served by any vehicle, assign a backup vehicle
    % for him/her.
    addK = sum(serviceTimes(iteration, :)==0); % The number of unserved customer.
    [~,id] = find(serviceTimes(iteration, :)==0); % Find index of unserved customer.
    addSeq = 1./zeros(addK, s);
    for k =1:addK
        addSeq(k, 1)=1;
        addSeq(k, 2)=id(1, addK);
        addSeq(k, 3)=s;
    end
    
    % Combine finalSeq and addSeq.
    finalSeq = [finalSeq;addSeq];
    actualUsedK = actualUsedK+addK;
    
    
    timeSeq = 1./zeros(actualUsedK, s);
    for k =1:actualUsedK
        timeSeq(k, 1) = 1;
        for n=2:s
            if finalSeq(k, n)==Inf
                break;
            end
            timeSeq(k, n) = 0;
            timeSeq(k, n) = timeSeq(k, n-1)+...
                C(finalSeq(k, n-1), finalSeq(k, n), 1, 2);
        end
    end
end

