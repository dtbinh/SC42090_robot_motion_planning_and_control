%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% main script
%
% Linda van der Spaa, TU Delft, 2018
% Original code by:
% Javier Alonso Mora, ETHZ, 2010
% Andreas Breitenmoser, MIT, 2009
% Mac Schwager, MIT, 2006
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Preparation
clear all
close all

% options
options = struct('animation',   1,...   % show animation
                 'plot',        1,...   % show plots
                 'lw',      2,...       % plot line width
                 'ms',      10,...      % plot marker size
                 'fs',      14);        % plot font size
lw = options.lw;
ms = options.ms;
fs = options.ms;

% parameters
par = struct('boundary', [0 0;0 1;1 1;1 0],...   % field boundary
             'N',           50,...       % numer of agents
             'res',         10,...       % centroid calulation resolution
             'dt',          0.1,...      % time step [s]
             'diffDrive',   true ,...    % Holonomic dynamcis (false) vs. differential Drive dynamics (true)
             'krho',        1,...        % Distance gain
             'kalpha',      1,...        % Gain on alpha
             'useConstantSpeed', true,...% Constant speed true/false
             'constantSpeed', 0.1);        % Desired constant speed
N = par.N;

patternTime = 2; % duration per goal pattern

% derived constants
xlb = min(par.boundary(:,1));
xub = max(par.boundary(:,1));
ylb = min(par.boundary(:,2));
yub = max(par.boundary(:,2));

% define obstacles through points of the polygon
% obstacle 1 (rectangle)
par.obstacles{1}=  [0.6,    0.35;
                    0.6,    0.2;
                    0.7,    0.2;
                    0.7,    0.35;
                    0.6,    0.35;];
% obstacle 2 (triangle)
par.obstacles{2}=  [0.1,    0.6;
                    0.1,    0.3;
                    0.4,    0.5;
                    0.1,    0.6;];
                
par.polys_obstacle1 = polyshape([par.obstacles{1}(:,1)]',[par.obstacles{1}(:,2)]');
par.polys_obstacle2 = polyshape([par.obstacles{2}(:,1)]',[par.obstacles{2}(:,2)]');


par.nObstacles  = 2;
par.obstaclesOn = true;

% initial conditions
rng('shuffle', 'twister');
if(par.diffDrive == false)
    s0 = rand(2*N,1); % x, y
else
    s0 = [rand(2*N,1);rand(N,1)*2*pi]; % x, y, phi 
end


% define obstacles through points of the polygon
% obstacle 1 (rectangle)
par.obstacles{1}=  [0.6,    0.35;
                    0.6,    0.2;
                    0.7,    0.2;
                    0.7,    0.35;
                    0.6,    0.35;];
% obstacle 2 (triangle)
par.obstacles{2}=  [0.1,    0.6;
                    0.1,    0.3;
                    0.4,    0.5;
                    0.1,    0.6;];
                
par.nObstacles  = 2;
par.obstaclesOn = true;

% check if initial position of robots are within obstacle
if par.obstaclesOn==true
    initial_position_ok=false;
    check=ones(N,1);
    while initial_position_ok==false
        [check,s0]=check_initial_position(s0,par);
        if all(check)==true
            initial_position_ok=true;
        end
    end
end


%% Pattern control
quick = false;
if(exist('main.mat')==0 && quick==false)
    zeroMass = cell(5,1);

    disp('Computing pattern 1 ...')
    pattern1 = simpleMassDistribution(1,   6.0e5, 0.05, 0.5, 1);
    [t1,s1,dens1,zm1] = Lloyd(pattern1, s0, [0 patternTime], par);
    zeroMass{1} = zm1;
    
    disp('Computing pattern 2 ...')
    pattern2 = simpleMassDistribution(3,   1.0e6, [0.1 0.1], [0.5 0.5], 0.1);
    [t2,s2,dens2,zm2] = Lloyd(pattern2, s1(:,end), [t1(end) 2*patternTime], par);
    zeroMass{2} = zm2;

    disp('Computing pattern 3 ...')
    pattern3 = simpleMassDistribution(2,   3.0e8, 0.1, [0.5 0.5], 1);
    [t3,s3,dens3,zm3] = Lloyd(pattern3, s2(:,end), [t2(end) 3*patternTime], par);
    zeroMass{3} = zm3;

    disp('Computing pattern 4 ...')
    pattern4 = simpleMassDistribution(0);
    [t4,s4,dens4,zm4] = Lloyd(pattern4, s3(:,end), [t3(end) 4*patternTime], par);
    zeroMass{4} = zm4;

    disp('Computing pattern 5 ...')
    pattern5 = imageMassDensity('stars.mat');
    [t5,s5,dens5,zm5] = Lloyd(pattern5, s4(:,end), [t4(end) 5*patternTime], par);
    zeroMass{5} = zm5;

    % combine
    t = [t1(1:end-1) t2(1:end-1) t3(1:end-1) t4(1:end-1) t5];
    s = [s1(:,1:end-1) s2(:,1:end-1) s3(:,1:end-1) s4(:,1:end-1) s5];

    save('main.mat','t1','s1','t2','s2','t3','s3','t4','s4','t5','s5','zeroMass')
elseif quick == false
    
    load('main.mat');
    disp('Computing pattern 5 ...')
    pattern5 = imageMassDensity('stars.mat',3);
    [t5,s5,dens5,zm5] = Lloyd(pattern5, s0, [0 2*patternTime], par);
    zeroMass{5} = zm5;
    
    % combine
%     t = [t1(1:end-1) t2(1:end-1) t3(1:end-1) t4(1:end-1) t5];
%     s = [s1(:,1:end-1) s2(:,1:end-1) s3(:,1:end-1) s4(:,1:end-1) s5];
    t = t5;
    s = s5;
else
    disp('Computing pattern 1 ...')
    pattern1 = imageMassDensity('stars.mat',2);
    [t1,s1,dens1,zm1] = Lloyd(pattern1, s0, [0 5*patternTime], par);
    zeroMass{1} = zm1;
    
    % combine
    t = [t1(1:end-1)];
    s = [s1(:,1:end-1)];
    
end

disp('Finished simulation')


%% Visualisation
x = s(1:N,:);
y = s(N+1:2*N,:);

%% animate robot trajectories
if options.animation
    disp('Preparing animation')
    animate(t,[x;y],par,options, zeroMass);
end

%% plot Voronoi 
if options.plot
    disp('Preparing Voronoi diagrams:')
    disp('    initial state')
    xinit = x(:,1);
    yinit = y(:,1);
    xinit = [xinit; -xinit; 2-xinit; xinit; xinit];
    yinit = [yinit; yinit; yinit; -yinit; 2-yinit];
    figure; plotVoronoi(xinit,yinit,options);
    axis equal;
    axis([xlb xub ylb yub]);
    
    disp('    final state')
    xfinal = x(:,end);
    yfinal = y(:,end);
    xfinal = [xfinal; -xfinal; 2-xfinal; xfinal; xfinal];
    yfinal = [yfinal; yfinal; yfinal; -yfinal; 2-yfinal];
    figure; plotVoronoi(xfinal,yfinal,options);
    
    % plot agent trajectories
    hold on;
    for i = 1:N
        plot(x(i,end), y(i,end), 'sk', 'LineWidth', lw, 'MarkerSize', ms);
        hold on;
        plot(x(i,:), y(i,:), '--k', 'LineWidth', lw, 'MarkerSize', ms);
    end
    axis equal;
    axis([xlb xub ylb yub]);
    hold on; 
    plot(par.obstacles{1}(:,1),par.obstacles{1}(:,2),'r','LineWidth',3);
    hold on;
    plot(par.obstacles{2}(:,1),par.obstacles{2}(:,2),'r','LineWidth',3);
end

%% plot density function
pattern = pattern5; dens = dens5;

[xCoord, yCoord] = meshgrid(xlb:0.01:xub, ylb:0.01:yub);
densDistribution = zeros(size(xCoord));
for i = 1:size(xCoord,1)
    for j = 1:size(xCoord,2)
        densDistribution(i,j) = getDens(xCoord(i,j), yCoord(i,j), pattern, par); % pattern(xCoord(i,j), yCoord(i,j));
    end
end
%imagesc(flipud(densDistribution)); axis square
%print -depsc2 figures/paths_obstacles_dist.eps

% normalize
maxMag = max(max(max(densDistribution)));
densDistribution = 1/maxMag*densDistribution;
% plot surface
figure; surf(xCoord, yCoord, densDistribution,'EdgeColor','none');
camlight left; lighting phong
% add robots' partitions at last time step
hold on
for k = 1:N
    stem3(dens(:,1,k,end), dens(:,2,k,end), 1/maxMag*dens(:,3,k,end), 'x');
end
axis equal;
axis([xlb xub ylb yub 0 1]);