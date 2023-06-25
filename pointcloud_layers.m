clear all
point = load("21_05_10_LST_georef.txt");
name = '21_05_10_LST_georef';

%clean the points over 8200 in X axis
k = point(:,1)<8100;
point = point(k,:);
%clean the points under 7850 in X axis
k = point(:,1)>7850;
point = point(k,:);
% %clean the points over 4100 in Y axis
k = point(:,2)<4100;
point = point(k,:);
%create point cloud
ptCloud = pointCloud(point(:,1:3));

X = ptCloud.Location(:,1);
Y = ptCloud.Location(:,2);
Z = ptCloud.Location(:,3);
x_min = min(X);y_min = min(Y);z_min = min(Z);

%survey points
sondaggi = load("Sondaggi_num.txt");
s = readlines('Sondaggi_BASE.txt');

s_1 = ptCloud.Location(:,1:2);
s_2 = sondaggi(:,1:2);
%find the nearest point in the point cloud
k = dsearchn(s_1,s_2); 

T = table(s(:,1),ptCloud.Location(k,1),ptCloud.Location(k,2),ptCloud.Location(k,3),'VariableNames', {'Survey', 'x', 'y', 'z'});

%surveys
storico = readtable('storico.txt');
storico.SURVEY = categorical(storico.SURVEY);
storico.FORMATION = categorical(storico.FORMATION);
T.Survey = categorical(T.Survey);

l = length(storico.SURVEY);
s = zeros(l,2);
%recompute surveys deep
for j=1:length(T.Survey)
    for i=1:length(storico.SURVEY)
        if (T.Survey(j)==storico.SURVEY(i))
            storico.MPC(i) = T.z(j) - storico.MPC(i);
            s(i,1)=T.x(j);
            s(i,2)=T.y(j);
        end
    end
end

X1 = s(:,1);
Y1 = s(:,2);
Z1 = storico.MPC;

% Clean contain level funciton
ptCloud = clean(0.1, ptCloud); %(Elevation Scale, point Cloud), es = 0.1 in the best for stip
ptCloud = pcrotate(0,0,-45,0,0,0,ptCloud, 'n');%(alpha, beta, gamma, x, y, z, point Cloud, plot y/n)

ptCloud = pcshow(ptCloud);
%layers
[X1,Y1,Z1] = level(X1,Y1,Z1,x_min,y_min,z_min);

%save point cloud
M = [X1 Y1 Z1];
ptCloud_M = pointCloud(M(:,1:3));
%DECOMMENT HERE TO SAVE POINTCLOUD
%pcwrite(ptCloud_M,'ptCloud.ply','Encoding','ascii');

%Create a transformation object with a 45 degree rotation along the z-axis.
rotationAngles = [0 0 -45];
translation = [0 0 0];
tform = rigidtform3d(rotationAngles,translation);

%Transform the point cloud.
ptCloud_M = pctransform(ptCloud_M,tform);

T = table(storico.SURVEY,storico.FORMATION,X1,Y1,Z1,'VariableNames', {'Survey', 'Formation', 'x', 'y', 'z'});
writetable(T, 'Sondaggi_z.txt')

X = ptCloud_M.Location(:,1);
Y = ptCloud_M.Location(:,2);
Z = ptCloud_M.Location(:,3);

%plot geological layer
clf
name = ["A","B1","C"];
for i=1:length(name)
    fmt = '%s';
    v = sprintf(fmt,name(i));
    ix = ismember(T.Formation,v);
    p = [X(ix) Y(ix) Z(ix)];
    %added
    fmt = '%s_sur.txt';
    v = sprintf(fmt,name(i));
    point = load(v);
    M = [p;point];
    ptCloud_f = pointCloud(M);
    pcshow(ptCloud_f.Location)
    hold on
    %end added
    S = table(M(:,1),M(:,2),M(:,3),'VariableNames', {'x', 'y', 'z'});
    fmt = '%s.txt';
    v = sprintf(fmt,name(i));
    writetable(S, v);
    X_f=M(:,1); Y_f=M(:,2); Z_f=M(:,3);
    TRI = delaunay(X_f,Y_f);
    % create triangulation in 3d
    tri = triangulation(TRI,X_f,Y_f,Z_f);
    % % patch('Faces',tri.ConnectivityList,'Vertices',tri.Points,'FaceColor','red')
    fmt = '%s_sur.stl';
    v = sprintf(fmt,name(i));
    stlwrite(tri,v,'text');
end