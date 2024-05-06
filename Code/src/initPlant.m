function initPlant(I_sim, axesFlag, dynamicsType, sequence)
modelDynamics = 'eulerPropagate';
modelKinematics = 'attitduePropagate';

load_system(modelDynamics)
dynMWS = get_param(modelDynamics, 'modelworkspace');
dynMWS.DataSource = 'MATLAB File';
dynMWS.FileName = 'plantConstants';
dynMWS.evalin('I_sim', I_sim)
dynMWS.evalin('axesFlag', axesFlag)
dynMWS.evalin('dynamicsType', dynamicsType)

load_system(modelKinematics)
kinMWS = get_param(modelKinematics, 'modelworkspace');
kinMWS.evalin('sequence', sequence)