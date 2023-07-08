%test for the solver

tspan = [0, 1];
h = 0.1;
y0 = 0;
[t,y] = solver(@testFunc, tspan, y0, h);

function dxdt = testFunc(t, x)

dxdt = x^4 - x^3 + x - 9;
end