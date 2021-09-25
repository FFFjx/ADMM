function [ UB ] = CalculateObject2UB( demand, lt, T, alpha2 )
%CALCULATEOBJECT2 此处显示有关此函数的摘要
%   此处显示详细说明
    c = size(demand,1);
    UB = 0;
    for i =1:c
        UB = UB+alpha2*(T-lt(i, 1))*demand(i, 1);
    end
end

