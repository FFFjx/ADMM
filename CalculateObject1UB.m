function [ UB ] = CalculateObject1UB( C )
%CALCULATEOJECT1UB �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    UB = sum(sum(C));
    UB = UB-sum(diag(C));
    UB = UB/2;

end

