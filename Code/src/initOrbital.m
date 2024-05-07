function initOrbital(orbitType, dataSource)
model = 'orbit_propagator';
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.DataSource = dataSource;
mws.FileName = 'orbitConstants';
mws.reload

mws.save('orbitConstants.mat')

mws.assignin('orbitType', orbitType)

save_system(model)