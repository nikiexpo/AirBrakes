function u = controller(z,goal)
    dx = goal;
    a = 1/dx^2;
    u = costFunction(z,a,goal);
end