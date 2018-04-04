function [none_in_obstacle,new_pos]=check_initial_position(old_pos,par)

    for ii=1:par.N
        start_in_obstacle=inObstacle(old_pos(ii),old_pos(par.N+ii), par);
        if start_in_obstacle==true
            %just change x coordinate if initial postion lies in obstacle
            new_pos(ii,1)=rand(1);
            none_in_obstacle(ii)=false;
        else
            new_pos(ii,1)=old_pos(ii);
            none_in_obstacle(ii,1)=true;
        end
        %keep y coordinate and heading anyways
        new_pos(par.N+ii,1)=old_pos(par.N+ii);
        new_pos(par.N*2+ii,1)=old_pos(par.N*2+ii);

    end
end