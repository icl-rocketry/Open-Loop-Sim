function [problem,guess,phaseoptions] = GoddardRocketThreePhase(vector)
%GoddardRocketThreePhase - Goddard Rocket Problem (Three-phase Formulation)
%
% The problem was adapted from Example 4.11 from
% J. Betts, "Practical Methods for Optimal Control and Estimation Using Nonlinear Programming: Second Edition," Advances in Design and Control, Society for Industrial and Applied Mathematics, 2010.
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk


% Initial and final time for different phases. Let t_min(end)=t_max(end) if tf is fixed.
problem.mp.time.t_min=[0 0 0];% 0 0];     
problem.mp.time.t_max=[0 70 70];% 70 70]; 
guess.mp.time=[0 10 20];% 30 45];

% Parameters bounds. pl=< p <=pu
problem.mp.parameters.pl=[0];
problem.mp.parameters.pu=[70];
guess.mp.parameters=[20];

% Bounds for linkage boundary constraints bll =< bclink(x0,xf,u0,uf,p,t0,tf,vdat) =< blu
problem.mp.constraints.bll.linear=[0 0 0  0 ];% 0 0 0  0  0  ];%0];
problem.mp.constraints.blu.linear=[0 0 0  0 ];% 0 0 0  0  0  ];%0];
problem.mp.constraints.blTol.linear=[0.01 0.01 0.01  0.01 ];% 0.01 0.01 0.01  0.01  0.01  ];% 0.01];

problem.mp.constraints.bll.nonlinear=[];
problem.mp.constraints.blu.nonlinear=[];
problem.mp.constraints.blTol.nonlinear=[];

% Get function handles
problem.mp.linkfunctions=@bclink;

% store the necessary problem parameters used in the functions
problem.mp.data = [];

% define different phases of OCP
[problem.phases{1},guess.phases{1}] = GoddardRocket_Phase1(problem.mp, guess.mp,vector);
[problem.phases{2},guess.phases{2}] = GoddardRocket_Phase2(problem.mp, guess.mp,vector);
phaseoptions{1}=settings_GoddardRocketThreePhase_Phase1(10);
phaseoptions{2}=settings_GoddardRocketThreePhase_Phase2(10);
% [problem.phases{3},guess.phases{3}] = GoddardRocket_Phase3(problem.mp, guess.mp);
% [problem.phases{4},guess.phases{4}] = GoddardRocket_Phase4(problem.mp, guess.mp);
% phaseoptions{3}=settings_GoddardRocketThreePhase_Phase1(10);
% phaseoptions{4}=settings_GoddardRocketThreePhase_Phase2(10);
%------------- END OF CODE --------------


function [blc_linear, blc_nonlinear]=bclink(x0,xf,u0,uf,p,t0,tf,vdat)

% bclink - Returns the evaluation of the linkage boundary constraints: bll =< bclink(x0,xf,u0,uf,p,t0,tf,vdat) =< blu
%
% Syntax:  [blc_linear, blc_nonlinear]=bclink(x0,xf,u0,uf,p,t0,tf,vdat)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    vdat- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    blc_linear - column vector containing the evaluation of the linear linkage boundary constraint functions
%    blc_nonlinear - column vector containing the evaluation of the nonlinear linkage boundary constraint functions
%
%------------- BEGIN CODE --------------

blc_linear=[xf{1}(1)-x0{2}(1);xf{1}(2)-x0{2}(2);xf{1}(3)-x0{2}(3);tf(1)-t0(2)];%  xf{3}(1)-x0{4}(1);xf{3}(2)-x0{4}(2);xf{3}(3)-x0{4}(3); tf(3)-t0(4); tf(3)-tf(2)-tf(1)];%;p(1)-tf(3)];
blc_nonlinear=[];
%------------- END OF CODE --------------
