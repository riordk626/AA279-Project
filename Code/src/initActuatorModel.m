function initActuatorModel(controlMoment, actuatorParams)
model = 'actuatorModel';

load_system(model)
mws = get_param(model, 'modelworkspace');
mws.assignin('controlMoment', controlMoment)

switch controlMoment
    case "reactionWheel"
        mws.assignin('Lw0', actuatorParams.Lw0)
        mws.assignin('A', actuatorParams.A)
end
