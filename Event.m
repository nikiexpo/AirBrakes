function [value,isterminal,direction] = Event(t,y)
    value = y(10);
    isterminal = 1;
    direction = 0;
end