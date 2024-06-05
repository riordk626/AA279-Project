function tsOut = targ2TsBus(R_om_des, Tfinal)

% elems(1) = Simulink.BusElement;
% elems(1).Name = 'R_des';
R_des_data = R_om_des.R_des;
R_des_data(:,:,2) = R_om_des.R_des;
R_des_ts = timeseries(R_des_data, [0 Tfinal]);

% elems(2) = Simulink.BusElement;
% elems(2).Name = 'om_des';
om_des_data = R_om_des.om_des;
om_des_data(:,2) = R_om_des.om_des;
om_des_ts = timeseries(om_des_data, [0 Tfinal]);

% tsOut = Simulink.Bus;
% tsOut.Elements = elems;

tsOut.R_des = R_des_ts;
tsOut.om_des = om_des_ts;