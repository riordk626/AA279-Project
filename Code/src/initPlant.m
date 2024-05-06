function initPlant(I_sim, axesFlag, dynamicsType, attitdueType, sequence, ICstruct)
modelDynamics = 'eulerPropagate';
modelKinematics = 'attitudePropagate';

load_system(modelDynamics)
dynMWS = get_param(modelDynamics, 'modelworkspace');
dynMWS.DataSource = 'MATLAB File';
dynMWS.FileName = 'plantConstants';
dynMWS.assignin('I_sim', I_sim)
dynMWS.assignin('axesFlag', axesFlag)
dynMWS.assignin('dynamicsType', dynamicsType)
dynMWS.assignin('om0', ICstruct.om0)

load_system(modelKinematics)
kinMWS = get_param(modelKinematics, 'modelworkspace');
kinMWS.assignin('attitdueType', attitdueType);
kinMWS.assignin('sequence', sequence)
switch attitdueType
    case "quat"
        q0 = RtoQuat(ICstruct.R0);
        kinMWS.assignin('q0', q0)
    case "euler"
        u0 = RtoEuler(ICstruct.R0, sequence);
        kinMWS.assignin('u0', u0)
end

dynMWS.reload
kinMWS.relaod

save_system(modelDynamics)
save_system(modelKinematics)