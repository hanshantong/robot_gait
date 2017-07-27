clear 

c = 0;
d = 1;

x = 0:0.001:1;
yp = d + sqrt(-c^2+2*x*c-x.^2+1);
ym = d - sqrt(-c^2+2*x*c-x.^2+1);

plot(c,d,'x')
hold on
grid on
plot(x,yp,'.')
plot(x,ym,'.')