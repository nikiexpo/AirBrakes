% SOLVER DETAILS: four-step Adams predictor-corrector method
% https://en.wikiversity.org/wiki/Adams-Bashforth_and_Adams-Moulton_methods
function [t, y] = solver(func, tspan, y0, h,data)
%timeseries t, timestep h
t = tspan(1):h:tspan(2);
a = length(y0); %rows
b = length(t); %cols
y = zeros(b,a);


%runge-kutta for the first 4 values
y(1, :) = y0;
t_rk = t(1:4);
for n = 1:(length(t_rk)-1)
    k1 = feval(func,t_rk(n), y(n,:), data);
    %disp(size(k1))
    k2 = feval(func,t_rk(n)+h/2, y(n,:)+h/2.*k1, data);
    k3 = feval(func, t_rk(n)+h/2, y(n,:)+h/2.*k2, data);
    k4 = feval(func,t_rk(n+1), y(n,:)+h.*k3, data);
    y(n+1, :) = y(n,:)+h.*(k1+2.*k2+2.*k3+k4)./6;
    %f =  y(n,:)+h.*(k1+2.*k2+2.*k3+k4)./6;
end


%adam-bashwidth for the rest
for n = 4:(length(t)-1)
    p = y(n,:) + h/24.*(55.*feval(func,t(n),y(n,:), data) - 59.*feval(func,t(n-1), y(n-1,:), data) + 37.*feval(func, t(n-2), y(n-2,:), data) - 9.*feval(func, t(n-3), y(n-3,:), data));
    y(n+1,:) = y(n,:) + h/24.*(9.*feval(func,t(n+1),p, data) + 19.*feval(func, t(n), y(n,:), data) - 5.*feval(func,t(n-1), y(n-1,:), data) + feval(func,t(n-2), y(n-2,:), data));
    if(y(n+1,10)<0)
        return;%terminate at apogee
    end
end


end

