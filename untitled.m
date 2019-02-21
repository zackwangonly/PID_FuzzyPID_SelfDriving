w1 = [1;0.77;0];
v = [20;40;60];
X = [ones(size(v)),v];
b = regress(w1,X);
w = logspace(-1,3,500);
x = [10,20,30,40,50,60];

%
