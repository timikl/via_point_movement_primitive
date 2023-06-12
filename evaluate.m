
function x,y = evaluate(a_vect,T_max,step)

a0 = a_vect(6)
a1 = a_vect(5)
a2 = a_vect(4)
a3 = a_vect(3)
a4 = a_vect(2)
a5 = a_vect(1)

t_linspace = linspace(0,T_max,step)
x = zeros(step)
y = zeros(step)

for i = linspace(0,step)
    t = t_linspace(i)
    x(i),y(i) = a5 .* t.^5 + a4 .* t.^4 + a3 .* t.^3 + a2 .* t.^2 + a1 .* t + a0;
end

%plot(linspace(0,t,100),polyval(a543210,linspace(0,t,100)))