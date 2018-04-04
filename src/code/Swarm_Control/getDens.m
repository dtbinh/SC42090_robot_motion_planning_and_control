function [dens] = getDens(x,y,massFcn,par)
%getDens Determines the densitiy based on the given mass distributution,
%taking into acount that the desnity is zero inside the obstacles.

if ~inObstacle(x,y,par)
    dens = massFcn(x,y);
else
    dens = 0;
end


