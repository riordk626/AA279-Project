function initDisturbance(disturbace, plantStuct)

model = 'distMoment';
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.assignin('disturbance', disturbace)

switch disturbace
    case "none"

    case "grav"
        distModel = 'gravityGradient';
        load_system(distModel)
        distMWS = get_param(distModel, 'ModelWorkspace');
        distMWS.assignin('I_sim', plantStuct.I_sim)
    case "mag"
        distModel = 'magneticField';
        load_system(distModel)
        distMWS = get_param(distModel, 'ModelWorkspace');
        distMWS.DataSource = 'MATLAB File';
        distMWS.FileName = 'magConstants';
        distMWS.reload
        distMWS.assignin("magnetic", "dipole")
end

% save_system(distModel)
% save_system(model)