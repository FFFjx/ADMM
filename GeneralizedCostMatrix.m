function [ GC ] = GeneralizedCostMatrix( C, alpha2, lt, demand, k1, k2 )
% According to cost matrix C and other parameters, use objective function 2 
% to generate generalized cost matrix GC. 
    spaceNodeNum = size(C, 1);
    T = size(C,3);
    GC = k1*C; % Generalized arc cost matrix.
    for i2 = 2:spaceNodeNum-1
        for t2 = lt(i2-1, 1)+1:T
            for i1 = 1:spaceNodeNum-1
                for t1 = 1:t2-1
                    GC(i1, i2, t1, t2) = GC(i1, i2, t1, t2) ...
                        + k2*alpha2*(t2-1-lt(i2-1, 1))*demand(i2-1, 1);
                end
            end
        end
    end
end

