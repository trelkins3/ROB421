function [y_dot] = motorDyn(t,y,p,v,t_v)
%FORWARDDYN Summary of this function goes here
%   Detailed explanation goes here

v_curr = interp1(t_v,v,t);

y_dot = ( v_curr - p(3)*y - (p(4)*y.^2 + p(2))*smooth_sign(y) )/p(1);

% hold on
% plot(t,y,'o')

end

function y_out = smooth_sign(x_in)

if( abs(x_in) < 0.001)
    x_in = 0;
end
y_out = sign(x_in);



end