function initKalmanFilter(Q, R, P0, dt_KF)
model = 'kalmanFilter';

load_system(model)
mws = get_param(model, 'modelworkspace');
mws.assignin('Q', Q)
mws.assignin('R', R)
mws.assignin('P0', P0)
mws.assignin('dt_KF', dt_KF)