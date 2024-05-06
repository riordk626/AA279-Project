function initDisturbance(disturbace)

model = 'distMoment';
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.assignin('disturbance', disturbace)

mws.reload

save_system(model)