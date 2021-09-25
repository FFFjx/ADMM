function [ B ]  =  GenerateRandomTime( customerNum, timeRange )
% Generate random delivery time in timeRange for each customers. Output
% matrix B is a (space nodes*space nodes) matrix, including beginning node
% and end node.
    spaceNodeNum = customerNum+2;
    lr = timeRange(1, 1);
    ur = timeRange(1, 2);
    B = zeros(spaceNodeNum, spaceNodeNum);
    for i = 1:spaceNodeNum
        for j = i+1:spaceNodeNum
            t = round(rand()*(ur-lr)+lr);
            B(i, j) = t;
            B(j, i) = t;
        end
    end
    B(1, spaceNodeNum) = 0;
    B(spaceNodeNum, 1) = 0;
    for k = 1:spaceNodeNum
       B(k, k) = 999; 
    end
    B4 = reshape(B, [size(B,1)*size(B,2), 1]);
    B1 = linspace(1, size(B4, 1), size(B4, 1));
    B1 = B1';
    B2 = linspace(0, spaceNodeNum-1, spaceNodeNum);
    B2 = repmat(B2, spaceNodeNum, 1);
    B3 = reshape(B2', [size(B2,1)*size(B2,2), 1]);
    B2 = reshape(B2, [size(B2,1)*size(B2,2), 1]);
    B = [B1,B2,B3,B4];

end

