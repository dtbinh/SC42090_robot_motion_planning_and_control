function [t,y,densityField,zmPoints] = Lloyd(massFcn, y0, tspan, par)

% Lloyd's algorithm for robot swarm control
%
% Inputs:
%   massFcn       function handle of probability density function encoding
%                 the desired swarm shape
%   y0            initial positions of the robot swarm [x;y] of size 2N x 1
%   tspan         [tinit tend] initial and final time of the simulation
%   par           parameters, required for this function:
%       N           number of robot in the swarm
%       dt          simulation timestep
%       res         numerical resolution
%
% Outputs:
%   t             simulation time vector
%   y             robot positions
%   densityField  probability density per area closest to each robot
%
% Author: Linda van der Spaa, TU Delft, 2018

N = par.N;
dt = par.dt;
t = tspan(1):dt:tspan(2);

positions = reshape(y0,[N size(y0,1)/N]);

y = zeros(size(y0,1),length(t));
densityField = zeros(par.res^2,3,N,length(t));
posZeroM = [];
for ti = 1:length(t)
    ti
    length(t)
    y(:,ti) = positions(:);
    inputs = zeros(N,2);
    
    for i = 1:N
        posRi = positions(i,1:2)';
        
        % calculate Voronoi
        V = Voronoi(positions(:,1:2), posRi, par);
        
        % calculate centroids
        [Mv, Lv, phi] = centroidNumerical(V, massFcn, par);
        densityField(:,:,i,ti) = phi;
        
        % robot control
        inputs(i,:) = controlLaw(positions(i,:)', Mv, Lv, par);
        
        if ti == 1 && Mv == 0 % Calculated zero mass over Voronoi region
            posZeroM = [posZeroM; posRi'];
        end
    end
    
    % update robot positions
    positions = positionUpdate(positions, inputs, par);
end
zmPoints = [tspan(:); posZeroM(:)];