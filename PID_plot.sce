// Read from file
data_PID = [];
t_PID = [];
F_PID = [];
z_PID = [];
stp_PID = [];

data_PID = read('PID C/data_PID_C.txt',-1,4);
t_PID = data_PID(:,1);
F_PID = data_PID(:,2);
z_PID = data_PID(:,3);
stp_PID = data_PID(:,4);

// Draw
subplot(212);
h = plot(t_PID, F_PID, 'b-', 'LineWidth',3);
ax=gca(),// get the handle on the current axes
ax.data_bounds=[0 -100;120 100];
set(gca(),"grid",[1 1]);
xlabel('t[s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('F [N]', 'font_style', 'times bold', 'font_size', 3);

subplot(211);
h = plot(t_PID, stp_PID, 'r-', t_PID, z_PID, 'b-', 'LineWidth',3);
ax=gca(),// get the handle on the current axes
ax.data_bounds=[0 0;120 250];
set(gca(),"grid",[1 1]);
l = legend('Setpoint', 'Response');
l.font_size = 3;
xlabel('t[s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('z [m]', 'font_style', 'times bold', 'font_size', 3);
