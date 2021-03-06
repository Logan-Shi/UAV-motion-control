surfix = './measure/';
filepath = strcat(surfix,'test.txt');
fi = fopen(filepath,'r');
formatSpec = '[%f,%f,%f]\n';
wayPts = fscanf(fi,formatSpec,[3,inf]);
h = 380;
wayPts(:,1:4) = wayPts(:,1:4) + [1208.5,230.1,1040.54 + h]';
wayPts(:,5:8) = wayPts(:,5:8) + [1208.5,230.1 - h,1040.54]';
fclose(fi);
savepath = strcat(surfix,'cube.mat');
save(savepath,'wayPts');