function initPlant(I_sim, axesFlag, dynamicsType, attitdueType, sequence, ICstruct)
modelDynamics = 'eulerPropagate';
modelKinematics = 'attitduePropagate';

load_system(modelDynamics)
dynMWS = get_param(modelDynamics, 'modelworkspace');
dynMWS.DataSource = 'MATLAB File';
dynMWS.FileName = 'plantConstants';
dynMWS.evalin('I_sim', I_sim)
dynMWS.evalin('axesFlag', axesFlag)
dynMWS.evalin('dynamicsType', dynamicsType)
dynMWS.evalin('om0', ICstruct.om0)

load_system(modelKinematics)
kinMWS = get_param(modelKinematics, 'modelworkspace');
kinMWS.evalin('attitdueType', attitdueType);
kinMWS.evalin('sequence', sequence)
switch attitdueType
    case "quat"
        kinMWS.evalin('q0', ICstruct.q0)
    case "euler"
        kinMWS.evalin('u0', ICstruct.u0)
end