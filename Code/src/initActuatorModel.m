function initActuatorModel(controlMoment, actuatorParams)
model = 'actuatorModel';

load_system(model)
mws = get_param(model, 'modelworkspace');
mws.assignin('controlMoment', controlMoment)

switch controlMoment
    case "reactionWheel"
        mws.assignin('Lw0', actuatorParams.Lw0)
        mws.assignin('A', actuatorParams.A)
        mws.assignin('Lwdot_max', actuatorParams.Lwdot_max)
        mws.assignin('Lw_max', actuatorParams.Lw_max)
        mws.assignin('sysWheel', actuatorParams.sys)
        % mws.assignin('num', actuatorParams.num)
        % mws.assignin('den', actuatorParams.den)
    case "magnetorquer"
        load("magConstants.mat", "mhat_ecef", "B0", "Re")
        mws.assignin('mhat_ecef', mhat_ecef)
        mws.assignin('B0', B0)
        mws.assignin('Re', Re)
end
