function initOrbital(orbitType)
model = 'orbit_propagator.slx';
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.DataSource = 'MATLAB File';
mws.FileName = 'orbitConstants';

mws.evalin('orbitType', orbitType)