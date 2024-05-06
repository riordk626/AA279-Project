function initDisturbance(disturbace)

model = 'distMoment';
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.evalin('disturbance', disturbace)