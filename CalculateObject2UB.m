function [ UB ] = CalculateObject2UB( demand, lt, T, alpha2 )
%CALCULATEOBJECT2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    c = size(demand,1);
    UB = 0;
    for i =1:c
        UB = UB+alpha2*(T-lt(i, 1))*demand(i, 1);
    end
end

