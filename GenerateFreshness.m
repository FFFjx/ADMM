function [ proposalR_K, freshnessDeviation ] = ...
    GenerateFreshness( finalSeq, timeSeq, freshness, R )
% According to the final visited sequence and corresponding time sequence,
% generate variable freshness for each vehicle. And calculate its freshness
% deviation.
    c = size(freshness, 1);
    s = c+2;
    K = size(finalSeq, 1);
    
    % Record cost value for each R with each vehicle.
    costObj3 = zeros(K, R);
    
    % Output. The vehicle freshness choice proposal.
    proposalR_K = zeros(K, 1);
    
    % Output. Calculate freshness deviation value according to vehicle
    % freshness choice proposal.
    freshnessDeviation = zeros(K, 1);
    for k=1:K
        for r=1:R
            cost = 0;
            for n=2:s
                if finalSeq(k,n)==s
                    break;
                end
                custID = finalSeq(k,n);
                lost = r*exp(0.005*(timeSeq(k, n)-timeSeq(k, 1)))-...
                    freshness(custID-1, 1);
                cost = cost+lost*lost;
            end
            costObj3(k, r) = costObj3(k, r)+cost;
        end
        minCost = min(costObj3(k, :));
        [~, minR] = find(costObj3(k, :)==minCost);
        freshnessDeviation(k, 1) = minCost;
        proposalR_K(k, 1) = minR;
    end
    
end

