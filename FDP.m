function [ completeRout ] = FDP( C, GC, profit, demand, bestK, W, flag )
% FDP is used to generate single vehicle least cost routing.
    s = size(GC, 1);
    T = size(GC, 3);
    
    % Record the uncompleted routings while doing the FDP.
    routing = 1./zeros(s, s+5, T+1);
    routingNum = zeros(1, T+1); % Record the number of routings at time t.
    
    % Record all the complete routing result and 3 kinds of cost. 
    completeRout = 1./zeros(s-1, s+3);
    
    % Initialize the begining node.
    % The 1:s column of routing store the sequence of visited nodes.
    routing(1, 1, 1) = 1; 
    
    % The (s+1)th column of (routing) store primal cost of this rout. (s+2)th
    % for ADMM cost. (s+3)th for LR cost.And the (s+4)th column store current
    % capacity.
    routing(1, s+1:s+4, 1) = 0; 
    
    % The (s+5)th column of (routing) store the number of visited nodes.
    routing(1, s+5, 1) = 1; 

    routingNum(1,1) = routingNum(1,1)+1;
    for t = 1:T
        % Variable flag is to determine ADMM cost(flag=0) or LR
        % cost(flag=1) is considered.
        if flag==0
            routing(:, :, t) = sortrows(routing(:, :, t), s+2);
        end
        if flag==1
            routing(:, :, t) = sortrows(routing(:, :, t), s+3);
        end
        
        for n=1:s
            if routing(n, s+1, t)==Inf
                routingNum(1, t) = n-1;
                break;
            end
        end
        % Consider the best K routings at time t.
        for rn = 1:(min(bestK,routingNum(1,t))) 
            visitedNum = routing(rn, s+5, t);
            fromNode = routing(rn, visitedNum, t);
            for tn = 1:s
                % Can't back to begining node.
                if tn==fromNode
                    continue;
                end
                toNode = tn;
                
                % Use matrix C to calculate next time. Arbitrarily t2>t1 is OK.
                nextTime = t+C(fromNode, toNode, 1, T); 
                
                % Over service time.
                if nextTime>T
                    continue;
                end
                
                % Reach the ending node, the routing is complete. Record in
                % variable completeRout.
                if toNode==s
                    newRout = reshape(routing(rn, 1:s+3, t), [1,s+3]);
                    newRout(1, visitedNum+1) = s;
                    newRout(1, s+1) = newRout(1, s+1)+C(fromNode, toNode, 1, T);
                    newRout(1, s+2) = newRout(1, s+2)+C(fromNode, toNode, 1, T);
                    newRout(1, s+3) = newRout(1, s+3)+C(fromNode, toNode, 1, T);
                    
                    % This is the most important part in FDP. Considered
                    % toNode=s(toNode is the ending node), then fromNode
                    % can be any other s-1 nodes. This is why the row of
                    % variable completeRout is (s-1). The variable
                    % completeRout(n,:) stores the least cost path while
                    % the end of the path is node n.
                    if flag==0
                        if newRout(1, s+2)<completeRout(fromNode,s+2)
                            completeRout(fromNode,:) = newRout;
                        end
                    end
                    if flag==1
                        if newRout(1, s+3)<completeRout(fromNode,s+3)
                            completeRout(fromNode,:) = newRout;
                        end
                    end
                    continue;
                end
                if toNode==1
                    continue;
                end
                
                % Check whether toNode is visited.
                if ismember(toNode,routing(rn, 1:s, t))==1
                    continue;
                end
                
                % Node toNode has not been visited before.
                if ismember(toNode,routing(rn, 1:s, t))==0
                    % Overweight.
                    if routing(rn, s+4, t)+demand(toNode-1, 1)>W
                        continue;
                    end
                    
                    % Feasible routing.
                    newRout = reshape(routing(rn, 1:s+5, t), [1,s+5]);
                    newRout(1, visitedNum+1) = toNode;
                    
                    newRout(1, s+1) = newRout(1, s+1)+GC(fromNode, toNode, t, nextTime);
                    newRout(1, s+2) = newRout(1, s+2)+GC(fromNode, toNode, t, nextTime)-profit(1, toNode-1);
                    newRout(1, s+3) = newRout(1, s+3)+GC(fromNode, toNode, t, nextTime)-profit(2, toNode-1);
                    newRout(1, s+4) = newRout(1, s+4)+demand(toNode-1, 1);
                    newRout(1, s+5) = newRout(1, s+5)+1;
                    
                    % Similar to above. routing(n,:) stores the least cost
                    % path while the end of the path is node n.
                    if flag==0
                        if newRout(1, s+2)<routing(fromNode, s+2, nextTime)
                            routing(fromNode, :, nextTime) = newRout;
                        end
                    end
                    if flag==1
                        if newRout(1, s+3)<routing(fromNode, s+3, nextTime)
                            routing(fromNode, :, nextTime) = newRout;
                        end
                    end
                    continue;
                end
            end
        end
    end

end