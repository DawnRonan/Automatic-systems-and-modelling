clc; clear; close all;

g1=5;
g2=[0.5 1]
G1 = tf(g1, g2) % system model
H1 = tf(0.7, 1);
sys1 = feedback(G1, H1);

sys2 = tf(10, 1);
sys3 = tf(1, [1, 0]);
sys0 = series(sys2,sys3);

sys4 = tf(0.5, 1)

sys5 = tf([5,1], [1,0]);
sys6 = tf(1,[1,0]);
sys0 = series(sys5,sys6);
sys5 = series(sys0,sys4);

sys0 = tf(1,1);
sys6 = feedback(sys0,sys5);

%generating the input
t=[0:0.01:15];
xref=ones(size(t));
%calculating step function
xout=lsim(sys6,xref,t);
plot(t,xout),grid;










n1=5;
d1=[0.5, 1];
sys1=tf(n1,d1);

n2=5;
d2=[1 10 25];
sys2=tf(n2,d2);
%1-term
n3=1;
d3=[1 0];
sys3=tf(n3,d3);
%inner loopsensor
n4=10;
d4=1;
sys4=tf(n4,d4);
%outer loopsensor
n5=1;
d5=1;
sys5=tf(n5,d5);
%inner loop elimination
sys6=feedback(sys2,sys4);
%feed forward path tf calculation
sys7=series(sys1,sys6);
sysff=series(sys7,sys3);
syscl=feedback(sysff,sys5)

%generating the input
t=[0:0.01:15];
xref=10*ones(size(t));
%calculating step function
xout=lsim(syscl,xref,t);
plot(t,xout),grid;
%error calculation
e=xref-xout;
%ploting
plot(t,xref,t,xout,t,e),grid
legend('xref','xout','e')
%dynamic performance
pzmap(syscl)