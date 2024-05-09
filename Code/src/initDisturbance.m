function initDisturbance(disturbace, plantStuct, dataSource)

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
        distMWS.DataSource = dataSource;
        distMWS.FileName = 'magConstants';
        distMWS.reload

        distMWS.save('magConstants.mat')

        distMWS.assignin("magnetic", "dipole")
    case "solar"
        distModel = 'solarMoment';
        load_system(distModel)
        distMWS = get_param(distModel, 'ModelWorkspace');
        distMWS.assignin('normalVectors', plantStuct.normalVectors)
        distMWS.assignin('areas', plantStuct.areas)
        distMWS.assignin('rcm', plantStuct.rcm)
        distMWS.assignin('centroids', plantStuct.centroids)
    case "aero"
        distModel = 'AerodynamicMoment';
        load_system(distModel)
        distMWS = get_param(distModel, 'ModelWorkspace');
        distMWS.assignin('normalVectors', plantStuct.normalVectors)
        distMWS.assignin('areas', plantStuct.areas)
        distMWS.assignin('rcm', plantStuct.rcm)
        distMWS.assignin('centroids', plantStuct.centroids)
end

% save_system(distModel)
% save_system(model)