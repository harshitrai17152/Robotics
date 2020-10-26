%%
clear;
clc;

%%
mdl_twolink;
twolink.base=([0,0,0]);

for t = 0:25
    twolink.plot([sin(0.5*t),cos(0.5*t)]);
end
