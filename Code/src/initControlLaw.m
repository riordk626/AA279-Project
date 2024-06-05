function initControlLaw(controlType, errorType, dt_cont, controllerParams)
model = 'controlLaw';

load_system(model)
mws = get_param(model, 'modelworkspace');

mws.assignin('dt_cont', dt_cont)
mws.assignin('controlType', controlType)
mws.assignin('errorType', errorType)
switch controlType
    case "PD"
        mws.assignin('kp', controllerParams.kp)
        mws.assignin('kd', controllerParams.kd)
end