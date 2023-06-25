point = load("21_05_10_LST_georef.txt");

%min e max
x_min = min(point(:,1));
x_max = max(point(:,1));
y_min = min(point(:,2));
y_max = max(point(:,2));
z_min = min(point(:,3));

%survey points
sondaggi = load("Sondaggi_num.txt");
s = readlines('Sondaggi_BASE.txt');
s_1 = point(:,1:2);
s_2 = sondaggi(:,1:2);
%find the nearest point in the point cloud
k = dsearchn(s_1,s_2); 

T = table(s(:,1),point(k,1),point(k,2),point(k,3),'VariableNames', {'Survey', 'x', 'y', 'z'});

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
            storico.MPC(i) = T.z(j)-storico.MPC(i);
            s(i,1)=T.x(j);
            s(i,2)=T.y(j);
        end
    end
end

X = s(:,1);
Y = s(:,2);
Z = storico.MPC;

if (x_min<0)
    X = X + (x_min*(-1));
else
    X = X - x_min;
end

if (y_min<0)
    Y = Y + (y_min*(-1));
else
    Y = Y - y_min;
end

if (z_min<0)
    Z = Z + (z_min*(-1));
else
    Z = Z - z_min;
end

k = X>50;
X = X(k);
Y = Y(k);
Z = Z(k);

%Create a transformation object with a 45 degree rotation along the z-axis.
rotationAngles = [0 0 -45];
translation = [0 0 0];
tform = rigidtform3d(rotationAngles,translation);

M_2 = [X, Y, Z];
ptCloud_2 = pointCloud(M_2);
%Transform the point cloud.
ptCloud_2 = pctransform(ptCloud_2,tform);

X = ptCloud_2.Location(:,1);
Y = ptCloud_2.Location(:,2);
Z = ptCloud_2.Location(:,3);

T = table(storico.SURVEY,storico.FORMATION,X,Y,Z,'VariableNames', {'Survey', 'Formation', 'x', 'y', 'z'});
writetable(T, 'Sondaggi_z.txt')

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
    point2 = point;
    point2(:,2) = point(:,2) + 100;
    M = [point;point2]; %p miss here
    ptCloud = pointCloud(M);
    pcshow(ptCloud.Location)
    %end added
    S = table(M(:,1),M(:,2),M(:,3),'VariableNames', {'x', 'y', 'z'});
    fmt = '%s.txt';
    v = sprintf(fmt,name(i));
    writetable(S, v);
    X=M(:,1); Y=M(:,2); Z=M(:,3);
    T = delaunay(X,Y);
    % create triangulation in 3d
    tri = triangulation(T,X,Y,Z);
    fmt = '%s_sur_1.stl';
    v = sprintf(fmt,name(i));
    stlwrite(tri,v,'text');
end

