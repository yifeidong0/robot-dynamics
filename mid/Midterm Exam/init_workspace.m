% Add folders to path
thisfile = which(mfilename);
exercisefolder = fileparts(thisfile);
cd(exercisefolder);
addpath(genpath(exercisefolder));
clear thisfile exercisefolder;

% Initialize the parameters for the mid-term exam.
init_params;
