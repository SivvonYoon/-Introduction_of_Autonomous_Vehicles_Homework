close all; clear; clc

%%

crowd = iread('wheres-wally.png', 'double');
wally = iread('wally.png', 'double');


S = WallySimilarity(wally, crowd, @zncc); % implement your function
[mx,p] = peak2(S, 1, 'npeaks', 5);
idisp(crowd);
plot_circle(p, 30, 'edgecolor', 'g') 
plot_point(p, 'sequence', 'bold', 'textsize', 24, 'textcolor', 'y')
