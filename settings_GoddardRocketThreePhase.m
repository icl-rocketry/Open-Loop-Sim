function options = settings_GoddardRocketThreePhase

%SETTINGS - General and solver-specific settings are selected here (multiphase)
% Unless specified otherwise the options are set using 0 => no and 1 => yes
%
% Syntax:  options = settings
%      
% Output:
%    options - Structure containing the settings
%
% Other m-files required: none
% Subfunctions: none
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

%------------- BEGIN CODE --------------


% Transcription Method:
%---------------------------------------
% Direct collocation method ('direct_collocation')
% Integral residual minimization method ('integral_res_min')
options.transcription='direct_collocation';

% integrated residual minimization solving method
% 'alternating';
% 'weightedCost'
options.min_res_mode='alternating';

% integrated residual minimization solution priority
% 'low_res_error';
% 'low_cost_value'
options.min_res_priority='low_res_error';

% Error criteria (in addition to constraint violation error)
% - local absolute error ('local_abs')      recommended for direct collocation
% - integral residual error ('int_res')     recommended for integral residual minimization
% - both ('both')                           most strict, convergence may be affected (Not Recommended)
options.errortype='int_res';


% Derivative generation :
%---------------------------------------
% Whenever the analytic differentiation is enabled it is necessary to
% specify the available analytic forms for the cost function, the dynamic equations 
% and  the constraints in the appropriate files .m

% Numerical differentiation: finite differences  ('numeric')
% Analytic differentiation: analytic gradients   ('analytic')
% Algorithmic differentiation with adigator  ('adigator'). Make sure you
% run startupadigator.m first in the adigator directory every time the MATLAB is restarted.
options.derivatives='numeric';
options.adigatorPath='../../adigator';


% Numeric generation of the Hessian:
%----------------------------------------------------------------

% Whenever the numeric differentiation is enabled it is necessary to
% specify which kind of finite difference approximation to use  between 
% the following ones:
% 
% Central difference ('central')  
options.hessianFD='central';


%  The perturbation size for numerical second derivatives 
%  can be set in options.perturbation.H. The perturbation size for numerical first derivatives 
%  can be set in  options.perturbation.J. 
%  It is possible to select default values for the perturbations by setting  options.perturbation.H and 
%  options.perturbation.J to the empty matrix.
%  The default values for the gradient approximation is (eps/2)^(1/3)
%  while for the  second derivative is (8*eps)^(1/3). 

options.perturbation.H=[];  % Perturbation size for the second derivatives
options.perturbation.J=[];  % Perturbation size for the first derivatives

% NLP solver
%---------------------------------------
% IPOPT: recommended but needs ipopt.mex        ('ipopt')
% fmincon                                       ('fmincon')
% WORHP                                         ('worhp')
options.NLPsolver='ipopt';

% IPOPT settings (if required)
options.ipopt.tol=1e-9;                        % Desired convergence tolerance (relative). The default value is  1e-8. 
options.ipopt.print_level=5;                   % Print level. The valid range for this integer option is [0,12] and its default value is 5.
options.ipopt.max_iter=5000;                   % Maximum number of iterations. The default value is 3000.
 
options.ipopt.mu_strategy ='adaptive';         % Determines which barrier parameter update strategy is to be used. 
                                               % The default value for this string option is "monotone".
                                               % Possible values:
                                               %   'monotone': use the monotone (Fiacco-McCormick) strategy
                                               %   'adaptive': use the adaptive update strategy

options.ipopt.hessian_approximation='exact';   %  Indicates what information for the Hessian of the Lagrangian function is                                                    
                                               %  used by the algorithm. The default value is 'exact'.
                                               %  Possible values:
                                               %   'exact': Use second derivatives provided by ICLOCS.
                                               %   'limited-memory': Perform a limited-memory quasi-Newton approximation
					                           %		             implemented inside IPOPT

options.ipopt.limited_memory_max_history=6;   % Maximum size of the history for the limited quasi-Newton Hessian approximation. The valid range for this integer option is [0, +inf) 
                                               % and its default value is 6. 
options.ipopt.limited_memory_max_skipping=1;  % Threshold for successive iterations where update is skipped for the quasi-Newton approximation.
                                               % The valid range for this integer option is [1,+inf) and its default value is 2. 

% fmincon settings (NOT RECOMMENDED!)
% See website for detailed info

% WORHP settings needed to be configured with the xml file

% Cold/Warm/Hot Start (recommended)
%---------------------------------------
options.start='Cold';

% Automatic scaling (recommended)
%---------------------------------------
options.scaling=1;

% Reorder of LGR Method
options.reorderLGR=0;

% Output settings
%---------------------------------------

% Display computation time
options.print.time=1;

% Display relative local discretization error (recommended for direct transcription)
options.print.relative_local_error=1;

% Display cost
options.print.cost=1;


% Plot figures
% 0: Do not plot
% 1: Plot all figures (state and input trajectory, multipliers/costate values and errors)
% 2: Plot only the state and input trajectory
% 3: Plot only the multipliers/costate values
% 4: Plot only the error values (absolute local error, relative local error and absolute constraint violation error)
options.plot=4;

% Meshing Strategy
% - fixed mesh ('fixed')
% - with local refinement of mesh ('mesh_refinement')
% - flexible mesh with adaptively spaced segments ('hp_flexible')
options.meshstrategy='mesh_refinement';

% Mesh Refinement Method
% Increase Polynomial Order        ('IO')
% Add intervals                    ('AI')
% Automatic refinement             ('Auto')
options.MeshRefinement='Auto';

% Mesh Refinement Preferences
%---------------------------------------
% Prioritize MR time               ('aggressive')   A relative aggressive scheme that aim to reduce the number of MR iterations and the total MR time
% Prioritize MR efficiency         ('efficient')   A relative aggressive scheme that aim to reduce the size of the problem at the end of MR iterations, making it potentially more efficient for online re-computations
options.MRstrategy='aggressive';

% Maximum number of mesh refinement iterations
options.maxMRiter=50;

% Early termination of residual minimization if tolerance is met
options.resminEarlyStop=0;



