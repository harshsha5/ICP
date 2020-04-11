function X = toSkewSym(x)
    
    %==== TODO: Output the 3-by-3 skew-symmetric matrix of the input 3-vector ====
    beta = x(1);
    gamma = x(2) ;
    alpha = x(3);
    % Write your code here...
    X = [0 -1*alpha gamma; alpha 0 -1*beta; -1*gamma beta 0];
%     X = [1 -1*alpha gamma; alpha 1 -1*beta; -1*gamma beta 1];
    
end
