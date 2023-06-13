function [y,dy,ddy] = polynomial2(a543210, dt, t)
% y = a5 * t^5 + a4 * t^4 + a3 * t^3 + a2 * t^2 + a1 * t + a0

y = zeros(2, int32(t/dt + 1)); % must be divisible
dy = zeros(2, int32(t/dt + 1)); % must be divisible
ddy = zeros(2, int32(t/dt + 1)); % must be divisible
% t must be the same as in polynomial1()

for i = 1:length(y)
  i_t = (i-1)*dt;
  y(i,1) = cell2mat(a543210(1)) .* i_t.^5 + cell2mat(a543210(2)) .* i_t.^4 + cell2mat(a543210(3)) .* i_t.^3 + ...
         cell2mat(a543210(4)) .* i_t.^2 + cell2mat(a543210(5)) .* i_t + cell2mat(a543210(6));
  dy(i) = 5 .* cell2mat(a543210(1)) .* i_t.^4 + 4 .* cell2mat(a543210(2)) .* i_t.^3 + ...
          3 .* cell2mat(a543210(3)) .* i_t.^2 + 2 .* cell2mat(a543210(4)) .* i_t + cell2mat(a543210(5));
  ddy(i) = 20 .* cell2mat(a543210(1)) .* i_t.^3 + 12 .* cell2mat(a543210(2)) .* i_t.^2 + ...
           6 .* cell2mat(a543210(3)) .* i_t + 2 .* cell2mat(a543210(4));
end
