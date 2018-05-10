[t,pwm,enc1,enc2,enc3] = importOpenLoop('steps.csv');
t = t/1000000;
v1 = 0.0254*enc1;
v2 = 0.0254*enc2;
v3 = 0.0254*enc3;
figure(3)
subplot(2,1,1)
hold off
plot(t,0.0254*enc1)
hold on
plot(t,0.0254*enc2)
plot(t,0.0254*enc3)
grid on
xlabel('Time (sec)');
ylabel('Speed (meters/sec)');
legend('Motor 1','Motor 2','Motor 3')

subplot(2,1,2)
hold off
plot(t,12/255*pwm)
grid on
xlabel('Time (sec)');
ylabel('Applied Voltage (V)');

%%
tau = 1/(40*pi); %%cut-off frequency [Hz]
sys = tf([1],[tau 1]); %%transfer function in continuous time

figure(2);
plot(t,v1);
t = linspace(t(1),t(end),length(t));
[v1] = lsim(sys,v1,t,0,'zoh');
hold on
plot(t,v1);


%%

v1_dot = -(-v1(1:(end-4)) + 8*v1(2:(end-3)) - 8*v1(4:(end-1)) + v1(5:end))/(12*mean(diff(t))); 

t_ = t(3:end-2);
[v1_dot] = lsim(sys,v1_dot,t_,0,'zoh');

figure;
plot(t_,v1_dot);

A = [ w_dot, w_', sign(w_')];
b = u_';

p = A\b;
p_lin = A(:,1:2)\b;
p_lin(3) = 0;
p_step = [.0027;0.0078;0];

fun = @(t_in,y_in) forwardDyn(t_in,y_in,p,u,t);
fun_lin = @(t_in,y_in) forwardDyn(t_in,y_in,p_lin,u,t);
fun_step = @(t_in,y_in) forwardDyn(t_in,y_in,p_step,u,t);

options = odeset('MaxStep',0.01);

[t_sim,y_sim] = ode23s(fun, t, w(1),options);