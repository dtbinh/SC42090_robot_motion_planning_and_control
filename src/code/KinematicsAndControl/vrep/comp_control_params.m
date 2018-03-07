%% This script contains code for processing the data for different runs with different control parameters
ii               = 2;
x_struct{ii}     = x_dat;
y_struct{ii}     = y_dat;
theta_struct{ii} = theta_dat;
vu_struct{ii}    = vu_dat;

% %% Alpha
% for jj = 1:ii
%     plot(x_struct{jj},y_struct{jj}); 
%     hold on;
%     axis square
% end
% 
% title('Effect of alpha on the trajectory');
% xlabel('x [m]');
% ylabel('y [m]');
% legend('alpha=1.5','alpha=2','alpha=3');

% %% Rho
% for jj = 1:ii
%     plot(x_struct{jj},y_struct{jj}); 
%     hold on;
%     axis square
% end
% 
% title('Effect of rho on the trajectory');
% xlabel('x [m]');
% ylabel('y [m]');
% legend('rho=0.1','rho=0.5','rho=0.7');

% %% Beta
% for jj = 1:ii
%     plot(x_struct{jj},y_struct{jj}); 
%     hold on;
%     axis square
% end
% 
% title('Effect of beta on the trajectory');
% xlabel('x [m]');
% ylabel('y [m]');
% legend('beta=-0.1','beta=-0.4','beta=-0.7');

% %% Backward allowed
% for jj = 1:ii
%     plot(x_struct{jj},y_struct{jj},'LineWidth',2); 
%     hold on;
%     axis square
% end
% % start position
% plot(0,1,'*k','LineWidth',2); 
% 
% 
% title('Effect of allowing backward motion on the trajectory');
% xlabel('x [m]');
% ylabel('y [m]');
% legend('backward = true','backward = false','Start','Location','NorthWest');
% xlim([-2.5,1.5]);
% ylim([-2.5,1.5]);

%% Constant velocity
close all
plot(x_struct{1},y_struct{1},'-','LineWidth',2); hold on
plot(x_struct{2},y_struct{2},'-','LineWidth',2); 
axis square

title('Effect of velocity scaling on the trajectory');
xlabel('x [m]');
ylabel('y [m]');
legend('const\_vel = true','const\_vel = false','Start','Location','NorthWest');
% xlim([-1.5,0.5]);
% ylim([-2.5,0.5]);

%% Velocity plot
close all
plot(vu_struct{1},'LineWidth',2); hold on;
plot(vu_struct{2},'LineWidth',2)
% ylim([0,0.5])
xlabel('Simulation step [-]');
ylabel('Velocity');
title('Velocity over time');
legend('Constant speed = true','Constant speed = false'); 
print -depsc2 'figures/vel.eps'