function initPlant(I_sim, axesFlag, dynamicsType, attitdueType, sequence, ICstruct)
modelDynamics = 'eulerPropagate';
modelKinematics = 'attitudePropagate';

load_system(modelDynamics)
dynMWS = get_param(modelDynamics, 'modelworkspace');
dynMWS.DataSource = 'MATLAB File';
dynMWS.FileName = 'plantConstants';
dynMWS.reload
dynMWS.assignin('I_sim', I_sim)
dynMWS.assignin('axesFlag', axesFlag)
dynMWS.assignin('dynamicsType', dynamicsType)
dynMWS.assignin('om0', ICstruct.om0)

load_system(modelKinematics)
kinMWS = get_param(modelKinematics, 'modelworkspace');
kinMWS.assignin('attitudeType', attitdueType);
kinMWS.assignin('sequence', sequence)
switch attitdueType
    case "quat"
        alpha0 = RtoQuat(ICstruct.R0);
        attModel = 'quaternionPropagate';
        icName = 'q0';
    case "euler"
        alpha0 = RtoEuler(ICstruct.R0, sequence);
        attModel = 'eulerAnglePropagate';
        icName = 'u0';
end

load_system(attModel)
attMWS = get_param(attModel, 'modelworkspace');
attMWS.assignin(icName, alpha0)

if attitdueType == "euler"
    attMWS.assignin("sequence", sequence)
end

save_system(attModel)
save_system(modelDynamics)
save_system(modelKinematics)