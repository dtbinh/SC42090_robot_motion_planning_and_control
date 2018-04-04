function [res] = inObstacle(x,y,par)
%inObstacle Determines wether a given point (x,y) is in any of the
%obstacles

res = false; % by default the result is false

if ~par.obstaclesOn || par.nObstacles <= 0
    return % early return if the obstacles ade disabled or if there are no obstacles
end

for ii = 1:par.nObstacles
    if(inpolygon(x,y,par.obstacles{ii}(:,1),par.obstacles{ii}(:,2)))
        res = true;
    end
end