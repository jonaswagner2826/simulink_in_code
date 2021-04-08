% simulink Generation Scipt
% Jonas Wagner
% 2021-04-08
% 
% Important References:
% ----------------------
% https://www.mathworks.com/help/simulink/slref/add_block.html
% https://www.mathworks.com/help/simulink/programmatic-modeling.html
% https://www.mathworks.com/help/simulink/referencelist.html
% https://www.mathworks.com/help/simulink/slref/common-block-parameters.html
% https://www.mathworks.com/help/simulink/slref/block-specific-parameters.html

clear
close all

%% Settings
generateModel = true;
openModel = true;
simulateModel = true;
plotResults = true;

% Name of the simulink model
subfolder = ''; %include / at end
fname = 'sim_model_default';


%% System Definitions (Simple double integrator system stabilization)
% Saturation Block
a = -1;
b = 1;

% Linearized System
A = [0, 1;
     0, 0];
B = [0;
     1];
C = eye(2);
D = [0;
     0];
lti_sys = ss(A,B,C,D);
x0 = [1;
     -1];


% Controller Gain
K = place(A,B, [-1+j,-1-j]);


if generateModel
%% Simulink Creation In Code
% Simulink Settings ----------------------
% Get the current configuration
cfg = Simulink.fileGenControl('getConfig');
% Changes Code Save Location
cfg.CacheFolder = [pwd, '\', subfolder];
cfg.CodeGenFolder = [pwd, '\', subfolder];
cfg.CodeGenFolderStructure = 'TargetEnvironmentSubfolder';
% Apply new Config
Simulink.fileGenControl('setConfig', 'config', cfg, 'keepPreviousPath',true, 'createDir',true);

% Check if the file already exists and delete it if it does
if exist(fname,'file') == 4
    % If it does then check whether it's open
    if bdIsLoaded(fname)
        % If it is then close it (without saving!)
        close_system(fname,0)
    end
    % delete the file
    delete([fname,'.slx']);
end

% Create Simulink Model
new_system;

% Create Simiple Input
add_block('simulink/Sources/In1', [gcs, '/In']);

% Create Sum block
add_block('simulink/Commonly Used Blocks/Sum', [gcs, '/Sum'],...
    'inputs', '|+-');

% Connections (Simple Input output 1 to Sum Block input 1)
add_line(gcs, 'In/1', 'Sum/1'); 

% Saturation Block
add_block('simulink/Commonly Used Blocks/Saturation', [gcs, '/Saturation'], ...
    'LowerLimit','a', ... % block parameters
    'UpperLimit','b');
add_line(gcs, 'Sum/1', 'Saturation/1');

% State-Space System
add_block('cstblocks/LTI System', [gcs, '/LTI_sys'],...
    'sys','lti_sys',...
    'IC', 'x0');
add_line(gcs, 'Saturation/1', 'LTI_sys/1');

% Controller (just a feedback gain)
add_block('simulink/Commonly Used Blocks/Gain', [gcs, '/Controller'],...
    'Gain', 'K',...
    'Multiplication', 'Matrix(K*u)',...
    'Orientation', 'left');
add_line(gcs, 'LTI_sys/1', 'Controller/1');
add_line(gcs, 'Controller/1', 'Sum/2');

% Create Simple Scope/Output
add_block('simulink/Sinks/Out1', [gcs, '/Out']);
add_line(gcs, 'LTI_sys/1', 'Out/1');


% Auto Arrange
Simulink.BlockDiagram.arrangeSystem(gcs) %Auto Arrange

%% Save and Open System
save_system(gcs,[subfolder, fname]);
print(['-s', gcs], '-dpng',... % Print model to figure
    [pwd, '\', subfolder, 'fig\', 'pblm3_c_model.png'])

end
if openModel
    open(fname); % Don't need to open to run
end

if simulateModel
%% Simulate System
simConfig.SaveState = 'on';
simOut = sim(fname, simConfig);

% Sim Data
Xout = simOut.xout{1}.Values.Data; %Only works by grabbing states of first block (LTI_sys)
end

if plotResults
%% Plot Results
fig = figure;
plot(Xout(:,1))
hold on
plot(Xout(:,2))
legend('X_1', 'X_2')
title('Saturated Double Integration Response while stabalized')
saveas(fig, [pwd, '\', subfolder, 'fig\', 'StateResponse.png'])
end

