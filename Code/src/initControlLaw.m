function initControlLaw(controlLaw, errorType, controllerParams)
model = 'controlLaw';

load_system(model)
mws = get_param(model, 'modelworkspace');

mws.assignin('errorType', errorType)
switch controlLaw
    case "PD"
        mws.assignin('kp', controllerParams.kp)
        mws.assignin('kd', controllerParams.kd)
end