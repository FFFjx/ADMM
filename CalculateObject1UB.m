function [ UB ] = CalculateObject1UB( C )
%CALCULATEOJECT1UB 此处显示有关此函数的摘要
%   此处显示详细说明
    UB = sum(sum(C));
    UB = UB-sum(diag(C));
    UB = UB/2;

end

