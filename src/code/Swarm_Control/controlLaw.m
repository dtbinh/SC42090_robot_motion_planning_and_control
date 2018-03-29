function u = controlLaw(posRi, Mv, Lv, par)

pos = posRi(1:2);

if(Mv==0)
        centroid = pos;
else
        centroid = (Lv/Mv);
end

if(par.diffDrive == false)
    u = 5*(centroid-pos);
else
    theta = posRi(end);
    xg = centroid(1);
    yg = centroid(2);
    x  = pos(1);
    y  = pos(2);
    
    % compute control quantities
    rho  = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
    lambda  = atan2(yg-y, xg-x);        % angle of the vector pointing from the robot to the goal in the inertial frame
    alpha   = lambda - theta;           % angle of the vector pointing from the robot to the goal in the robot frame
    alpha   = normalizeAngle(alpha);

    vu      = par.krho * rho;
    omega   = par.kalpha * alpha; 

    if(par.useConstantSpeed == true)
        % Scale vu and omega
        vscale  = par.constantSpeed/vu;
        vu      = vu*vscale;
        omega   = omega*vscale;
    end
    
    if(rho > 0.005)
        u       = [vu; omega];
    else % Zeros forward speed if the robot is within a certain range of the target
        u       = [0; 0];
    end
    

end
