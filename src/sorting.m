%% Sort Function 
% sorts color to approprate space
function sorting(color,cam)
% start by getting x y values of object
while(1)
[x,y] = Imagefindandprocess(color,cam);
if(x==0 && y==0)
    warning('No more in this color leaving loop');
    break;
else % continue on with function
    % move to object and return torque
    t = MoveandWeighobject(x,y);
    % dummy parameters
    if (2 < t(3)) && (t(3) < 25)
        % heavy object
    a = 1;
    else %light object
        a = 0;
    end
    % go to sorted area
    sortbycolorandweight(a,color);
end
end
end