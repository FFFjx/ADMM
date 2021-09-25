function [ visitedNode ] = CalculateCostForVehicle( visitedSeq, C, GC )
% This function is used to calculate primal cost for each vehicle in a
% single iteration. Input visitedSeq should be a [K, s] matrix. Output
% visitedNode is a [K, s+1] matrix. The (s+1)th column stores the total
% primal cost.
    K = size(visitedSeq, 1);
    s = size(visitedSeq, 2);
    visitedNode = zeros(K, s+1);
    visitedNode(:,1:s) = visitedSeq(:,:);

    for k=1:K
        nextTime=1;
        for n=2:s
            thisTime = nextTime;
            nextTime = nextTime+C(visitedNode(k, n-1),visitedNode(k, n),1,2);
            visitedNode(k, s+1) = visitedNode(k, s+1)+...
                GC(visitedNode(k, n-1),visitedNode(k, n), thisTime, nextTime);
            if visitedNode(k, n)==s
                break;
            end
        end
    end

end

