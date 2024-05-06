function initDisturbance(disturbace, plantStuct)

model = 'distMoment';
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.assignin('disturbance', disturbace)

switch disturbace
    case "grav"
        gravModel = 'gravityGradient';
        load_system(gravModel)
        gravMWS = get_param(gravModel, 'ModelWorkspace');
        gravMWS.assignin('I_sim', plantStuct.I_sim)
end

save_system(gravModel)
save_system(model)