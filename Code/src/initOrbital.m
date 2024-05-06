function initOrbital(orbitType)
model = 'orbit_propagator';
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.DataSource = 'MATLAB File';
mws.FileName = 'orbitConstants';

mws.assignin('orbitType', orbitType)
mws.reload

save_system(model)