[t,pwm,enc1,enc2,enc3] = importOpenLoop('dual.csv');
figure(1)
plot(pwm,enc1,'.')

%%
figure(2)
hold off
% plot(pwm)
plot(t,pwm)
hold on
N = 100;
plot(t(N+1:end),pwm(N+1:end)-pwm(1:end-N),'.')
plot(t,enc1)

figure(4)
ss = zeros(size(t));
ss(101:end) = (pwm(N+1:end)-pwm(1:end-N))==0;
ss = logical(ss);
plot(abs(enc1(ss)),abs(pwm(ss)),'.')
hold on
plot(abs(enc2(ss)),abs(pwm(ss)),'.')
plot(abs(enc3(ss)),abs(pwm(ss)),'.')
xlabel('Applied Voltage [v]');
ylabel('Steady State Speed (meter/sec)')
legend('Motor 1','Motor 2','Motor 3')



f1 = fit(abs(enc1(ss)),abs(pwm(ss)),'exp2')
f2 = fit(abs(enc2(ss)),abs(pwm(ss)),'exp2')
f3 = fit(abs(enc3(ss)),abs(pwm(ss)),'exp2')

hold on
plot(f1)
plot(f2)
plot(f3)

%%

figure(3)
subplot(2,1,1)
hold off
plot(t/1000000,0.0254*enc1)
hold on
plot(t/1000000,0.0254*enc2)
plot(t/1000000,0.0254*enc3)
grid on
xlabel('Time (sec)');
ylabel('Speed (meters/sec)');
legend('Motor 1','Motor 2','Motor 3')

subplot(2,1,2)
hold off
plot(t/1000000,12/255*pwm)
grid on
xlabel('Time (sec)');
ylabel('Applied Voltage (V)');

