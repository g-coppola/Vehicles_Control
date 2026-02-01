clear
close all
clc

load('H2_Data.mat')

[scenario, egoVehicle, actors] = ScenarioH2(x_sim,y_sim,psi_sim,v_sim);

figure
tiledlayout(2, 1, 'TileSpacing', 'compact');

ax1 = nexttile; 
plot(scenario, 'Parent', ax1, 'Meshes', 'on'); 
view(ax1, 2); 
axis(ax1, 'equal'); 

ax2 = nexttile; 
chasePlot(egoVehicle, 'Parent', ax2, 'Meshes', 'on'); 
grid(ax2, 'on');

while advance(scenario)
    pause(0);
end

clear
clc

