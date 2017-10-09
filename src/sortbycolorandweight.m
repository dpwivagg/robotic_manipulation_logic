%% sort by color and weight
%
% 
function sortbycolorandweight(weight,color)

if (weight == 1)
    % heavy object go to right
     y = -16;
else % light object go to left
    y= 16;
end
if strcmp(color, 'yellow')
         x = 5;
else if strcmp(color, 'green')
         x = 0;
    else % blue
          x = -5;
         end
end
     % now move and drop to sort
MoveandDrop(x,y);

end