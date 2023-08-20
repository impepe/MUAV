Ts = 1/100;
Gs = tf([1802380118.95],[1,100,50])
Gz = c2d(Gs,Ts,'zoh');
sisotool(Gz)