% main_GoddardRocketThreePhase - Main script to solve the Optimal Control Problem with a multi-phase formulation
%
% Goddard Rocket Problem (Three-phase Formulation)
%
% The problem was adapted from Example 4.9 from
% J. Betts, "Practical Methods for Optimal Control and Estimation Using Nonlinear Programming: Second Edition," Advances in Design and Control, Society for Industrial and Applied Mathematics, 2010.
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk

function []=mainfunct(vector)

rho0=vector(1); %sea level density NOMINAL VALUE = 1.225kg/m^3
Cd=vector(2); %drag coefficient NOMINAL VALUE = 0.5 
diameter=vector(3); %diameter of the rocket section NOMINAL VALUE = 0.1778
Sref=pi*vector(3)^2/4; %Wetted area m^2
D0=0.5*vector(1)*Sref*vector(2); %Drag term independent of altitude change NOMINAL VALUE = 0.0076
H=2.252.*10.^(-5); %function by which the desnity changes as a function of altitude
T_min=0;
T_max=vector(4); %max thrust produced NOMINAL VALUE = 1000N
grav=9.81; %geopotential gravity acceleration
c=vector(5); %I_sp specific impulse NOMINAL VALUE = 2000N


% initial conditions of the state vector x_0=[0 0 45]
h_0=0; %m
v_0=0; %m/s
m_0=45; %kg

h_f=3048; %target altitude for competition
m_f=vector(6); %dry mass NOMINAL VALUE = 26kg

h_max=3400; %upper bound 
v_max=2500;%upper bound
m_max=45; %dry mass will not change

h_min=0; %no negative altitude!
v_min=-2500; %obvious
m_min=vector(6); %mass cannot be less than dry mass


%--------------------------------------------------------

% clear all;close all;format compact;

options.mp= settings_GoddardRocketThreePhase;                  % Get options and solver settings 
[problem,guess,options.phaseoptions]=GoddardRocketThreePhase(vector);          % Fetch the problem definition
[solution,MRHistory]=solveMyProblem( problem,guess,options);

%%
for i=1:length(solution.phaseSol)
    sol=solution.phaseSol{i};
    xx=linspace(sol.t0,sol.tf,1000);
    
    
    figure(100)
    hold on
    plot(xx-(i>2)*solution.phaseSol{2}.tf,speval(sol,'X',1,xx),'linewidth',2)
    xlabel('Time [s]')
    ylabel('Altitude [ft]')
    grid on
    
    figure(101)
    hold on
    plot(xx-(i>2)*solution.phaseSol{2}.tf,speval(sol,'X',2,xx),'linewidth',2)
    xlabel('Time [s]')
    ylabel('Velocity [ft/s]')
    grid on
    
    figure(102)
    hold on
    plot(xx-(i>2)*solution.phaseSol{2}.tf,speval(sol,'X',3,xx),'linewidth',2)
    xlabel('Time [s]')
    ylabel('Mass [lbm]')
    grid on
    
    figure(103)
    hold on
    plot(xx-(i>2)*solution.phaseSol{2}.tf,speval(sol,'U',1,xx),'linewidth',2)
    ylim([problem.phases{2}.inputs.ul problem.phases{1}.inputs.uu])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input (Thrust) [lbf]')

end



    