aircraft_parameters.g = 9.8;
   
%physical parameters of airframe
aircraft_parameters.m = 13.5; %[kg]
aircraft_parameters.W = aircraft_parameters.m*aircraft_parameters.g; %[N]
aircraft_parameters.Ix   = 0.8244; %[kg m2]
aircraft_parameters.Iy   = 1.135; %[kg m2]
aircraft_parameters.Iz   = 1.759; %[kg m2]
aircraft_parameters.Ixz  = 0.1204; %[kg m2]


%Aircraft parameters
aircraft_parameters.c             = 0.18994; %[m]
aircraft_parameters.b             = 2.8956; %[m]
aircraft_parameters.S             = 0.55; %[m2]
aircraft_parameters.AR            = aircraft_parameters.b*aircraft_parameters.b/aircraft_parameters.S;

%%aircraft_parameters.height        = 200; % Altitude [m] ?
%%aircraft_parameters.rho           = 1.2682; % Air density [kg/m3]

%aircraft_parameters.Mach = aircraft_parameters.u0/aircraft_parameters.a;
%aircraft_parameters.a = 1093.2;         % Speed of sound

% Aerodynamic coefficients
aircraft_parameters.e             = 0.9; %  Oswald's efficiency factor
aircraft_parameters.M             = 50; % What is this??
aircraft_parameters.epsilon       = 0.1592;

aircraft_parameters.K = 1/(aircraft_parameters.e*aircraft_parameters.AR*pi); %%%%%% EWF

% Motor parameters
aircraft_parameters.Sprop        = 0.2027;
aircraft_parameters.kmotor       = 50;
aircraft_parameters.Cprop        = 1;


% Lift coefficients
aircraft_parameters.CL0         = 0.28;
aircraft_parameters.CLmin       = 0.28;%%%%%% EWF
aircraft_parameters.CLalpha     = 3.45;
aircraft_parameters.CLalphadot  = 0.0;
aircraft_parameters.CLq         = 0.0;
aircraft_parameters.CLde        = -0.36;

% Drag coefficients
aircraft_parameters.CDpa        = 0.0437;
aircraft_parameters.CDmin        = 0.0437;%%%%%% EWF
aircraft_parameters.CD0         = 0.03;
aircraft_parameters.CDalpha     = 0.30;
aircraft_parameters.CDq         = 0.0;
aircraft_parameters.CDde        = 0.0;

% Pitch moment coefficients
aircraft_parameters.Cm0         = -0.02338;
aircraft_parameters.Cmalpha     = -0.38;
aircraft_parameters.Cmalphadot  = 0;
aircraft_parameters.Cmq         = -3.6;
aircraft_parameters.Cmde        = -0.5;

% Sideforce coefficients
aircraft_parameters.CY0         = 0.0;
aircraft_parameters.CYbeta      = -0.98;
aircraft_parameters.CYp         = 0.0;
aircraft_parameters.CYr         = 0.0;
aircraft_parameters.CYda        = 0.0;
aircraft_parameters.CYdr        = -0.17;

% Roll moment coefficients
aircraft_parameters.Cl0         = 0.0;
aircraft_parameters.Clbeta      = -0.12;
aircraft_parameters.Clp         = -0.26;
aircraft_parameters.Clr         = 0.14;
aircraft_parameters.Clda        = 0.08;
aircraft_parameters.Cldr        = 0.105;

% Yaw moment coefficients
aircraft_parameters.Cn0         = 0.0;
aircraft_parameters.Cnbeta      = 0.25;
aircraft_parameters.Cnp         = 0.022;
aircraft_parameters.Cnr         = -0.35;
aircraft_parameters.Cnda        = 0.06;
aircraft_parameters.Cndr        = -0.032;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Additional terms needed for linear models
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
inertia_terms = InertiaTerms(aircraft_parameters);

aircraft_parameters.Cp0 = inertia_terms(3)*aircraft_parameters.Cl0 + inertia_terms(4)*aircraft_parameters.Cn0;
aircraft_parameters.Cpbeta = inertia_terms(3)*aircraft_parameters.Clbeta + inertia_terms(4)*aircraft_parameters.Cnbeta;
aircraft_parameters.Cpp = inertia_terms(3)*aircraft_parameters.Clp + inertia_terms(4)*aircraft_parameters.Cnp;
aircraft_parameters.Cpr = inertia_terms(3)*aircraft_parameters.Clr + inertia_terms(4)*aircraft_parameters.Cnr;
aircraft_parameters.Cpda = inertia_terms(3)*aircraft_parameters.Clda + inertia_terms(4)*aircraft_parameters.Cnda;
aircraft_parameters.Cpdr = inertia_terms(3)*aircraft_parameters.Cldr + inertia_terms(4)*aircraft_parameters.Cndr;

aircraft_parameters.Cr0 = inertia_terms(4)*aircraft_parameters.Cl0 + inertia_terms(8)*aircraft_parameters.Cn0;
aircraft_parameters.Crbeta = inertia_terms(4)*aircraft_parameters.Clbeta + inertia_terms(8)*aircraft_parameters.Cnbeta;
aircraft_parameters.Crp = inertia_terms(4)*aircraft_parameters.Clp + inertia_terms(8)*aircraft_parameters.Cnp;
aircraft_parameters.Crr = inertia_terms(4)*aircraft_parameters.Clr + inertia_terms(8)*aircraft_parameters.Cnr;
aircraft_parameters.Crda = inertia_terms(4)*aircraft_parameters.Clda + inertia_terms(8)*aircraft_parameters.Cnda;
aircraft_parameters.Crdr = inertia_terms(4)*aircraft_parameters.Cldr + inertia_terms(8)*aircraft_parameters.Cndr;



