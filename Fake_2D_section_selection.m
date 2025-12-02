clear 
clc 
close all 
%T = readmatrix("C:\Users\Francion_Specialist2\Documents\Civitella\Civitella_mesh_filtered.txt"); 
filePath = "C:\Users\Francion_Specialist2\Documents\Conero\Points_Piangrande_2.txt"; 
T = readmatrix(filePath); 
 
%% PARAMETER SELECTION UI 
% Call the UI to select parameters 
params = parameterSelectionUI(); 
 
gridStep = params.gridStep; 
max_edge_length = params.max_edge_length; 
z_deep = params.z_deep; 
interpolation = params.interpolationMethod; 
extrapolation = params.extrapolationMethod; 
 
% Distance between cutting planes (in meters) 
planeDistance = params.planeDistance; % Distance between the two parallel planes 
 
% Split into folder, name, extension 
[folder, name, ~] = fileparts(filePath); 
% Build new filename: original_name + "_srf" + original_extension 
stl_out = fullfile(folder, name + ".stl"); 
 
%% 
% Create the point cloud 
ptCloud = pointCloud([T(:,1) T(:,2) T(:,3)]); 
 
%% 
% Create a logical index for values not equal to -9999 
validIndices = T(:,3) >= -9999; 
%% 
% Filter the point cloud 
filteredPtCloud = pointCloud(ptCloud.Location(validIndices, :)); 
 
figure(1) 
pcshow(filteredPtCloud); 
title('DTM'); 
xlabel('X-axis'); 
ylabel('Y-axis'); 
zlabel('Z-axis'); 
 
%% INTERACTIVE PLANE SELECTION (MOVED HERE - RIGHT AFTER READING POINT CLOUD) 
% Get initial point cloud data 
x_initial = filteredPtCloud.Location(:,1); 
y_initial = filteredPtCloud.Location(:,2); 
z_initial = filteredPtCloud.Location(:,3); 
 
% Call interactive plane selector with editable distance 
[planePosition, planeNormal, planeDistance, targetAxis] = interactivePlaneSelector(x_initial, y_initial, z_initial, planeDistance); 
 
% Enforce vertical planes (normal z-component = 0) 
if abs(planeNormal(3)) > 1e-6 
    fprintf('Warning: Plane was not vertical. Forcing verticality.\n'); 
    planeNormal(3) = 0; 
    planeNormal = planeNormal / norm(planeNormal); 
end 
 
% Calculate Dip and Dip Direction 
calcNormal = planeNormal; 
if calcNormal(3) < 0 
    calcNormal = -calcNormal; 
end 
dip = acosd(calcNormal(3)); 
% Calculate math angle from X axis (CCW) 
theta = atan2d(calcNormal(2), calcNormal(1)); 
% Convert to Azimuth (Clockwise from North) 
dipDir = mod(90 - theta, 360); 
 
% Second plane position 
planePosition2 = planePosition + planeDistance * planeNormal; 
 
% Cut the initial point cloud between the two planes 
planePosExpanded = repmat(planePosition, length(x_initial), 1); 
normalExpanded = repmat(planeNormal, length(x_initial), 1); 
d1 = dot([x_initial, y_initial, z_initial] - planePosExpanded, normalExpanded, 2); 
d2 = dot([x_initial, y_initial, z_initial] - (planePosExpanded + planeDistance * normalExpanded), normalExpanded, 2); 
 
% Keep points between the two planes 
cutIndices = (d1 >= 0) & (d2 <= 0); 
 
fprintf('Original points: %d\n', length(x_initial)); 
fprintf('Points after plane cutting: %d\n', sum(cutIndices)); 
 
% Apply the cut to the point cloud 
filteredPtCloud = pointCloud(filteredPtCloud.Location(cutIndices, :)); 
 
%% ROTATION AROUND Z-AXIS ONLY - TO ALIGN WITH X OR Y AXIS 
% Goal: Rotate the point cloud around Z-axis to align strike direction with X or Y axis 
% using the plane dip and dip direction. 
 
% Convert target axis to Azimuth 
targetAz = []; 
if ischar(targetAxis) || isstring(targetAxis) 
    if strcmpi(targetAxis, 'X') || strcmpi(targetAxis, 'X-axis') 
        targetAz = 90; % East 
    elseif strcmpi(targetAxis, 'Y') || strcmpi(targetAxis, 'Y-axis') 
        targetAz = 0; % North 
    end 
end 
 
if isempty(targetAz) 
    % If 'Minimal' or unrecognized, do not apply alignment rotation 
    R = eye(3); 
    fprintf('Target axis is Minimal or unset - no alignment rotation applied\n'); 
else 
    % Calculate Strike Azimuth from Dip Direction 
    % Strike is 90 degrees left of Dip Direction (Right Hand Rule)? 
    % Or simply perpendicular. 
    % Standard convention: Strike is 90 deg counter-clockwise from Dip Direction? 
    % Wait. Dip Direction is 90 deg clockwise from Strike (Right Hand Rule). 
    % So Strike = DipDir - 90. 
    %strikeAz = mod(dipDir, 360); 
    strikeAz = dipDir; 
 
    % Calculate rotation required (CCW) 
    % NewAz = OldAz - RotationCCW 
    % TargetAz = StrikeAz - RotationCCW 
    % RotationCCW = StrikeAz - TargetAz 
    rotation_angle_deg = targetAz - strikeAz; 
    
    % Convert to radians 
    angle_z = deg2rad(rotation_angle_deg); 
 
    % Rotation matrix around Z-axis 
    R = [cos(angle_z), -sin(angle_z), 0; 
         sin(angle_z),  cos(angle_z), 0; 
         0,             0,             1]; 
      
    fprintf('Aligning Strike (%.2f°) to Target (%.2f°)\n', strikeAz, targetAz); 
    fprintf('Rotation around Z-axis: %.2f degrees\n', rotation_angle_deg); 
end 

% Save to file 
paramFile = fullfile(folder, name + "_Plane_coords.txt"); 
fid = fopen(paramFile, 'w'); 
if fid ~= -1 
    fprintf(fid, 'Plane 1:\n'); 
    fprintf(fid, 'Position (X, Y, Z): %.4f, %.4f, %.4f\n', planePosition(1), planePosition(2), planePosition(3)); 
    fprintf(fid, 'Dip: %.4f degrees\n', dip); 
    fprintf(fid, 'Dip Direction: %.4f degrees\n', dipDir); 
    fprintf(fid, '\nPlane 2:\n'); 
    fprintf(fid, 'Position (X, Y, Z): %.4f, %.4f, %.4f\n', planePosition2(1), planePosition2(2), planePosition2(3)); 
    fprintf(fid, 'Dip: %.4f degrees\n', dip); 
    fprintf(fid, 'Dip Direction: %.4f degrees\n', dipDir); 
    fprintf(fid, '\nDistance: %.4f meters\n', planeDistance); 
    fprintf(fid, '\nPoint Cloud Rotation angle: %.4f degrees\n', rotation_angle_deg); 
    fprintf(fid, 'Target Axis: %s\n', targetAxis); 
    fclose(fid); 
    fprintf('Plane coordinates saved to %s\n', paramFile); 
else 
    fprintf('Error: Could not open %s for writing.\n', paramFile); 
end 
 
 
% Create the affine transformation 
affine_matrix = eye(4); 
affine_matrix(1:3, 1:3) = R'; % Use transpose for pctransform (row vectors) 
tform_align = affinetform3d(affine_matrix); 
 
% Apply the transformation 
ptCloud_rotated = pctransform(filteredPtCloud, tform_align); 
 
% Store the inverse transformation for rotating back later 
tform_back = invert(tform_align); 
 
%% OPTIONAL MANUAL ROTATION 
% Allow user to perform an additional rotation interactively 
tform_manual = interactiveRotationSelector(ptCloud_rotated); 
ptCloud_rotated = pctransform(ptCloud_rotated, tform_manual); 

% Update the inverse transform to include the manual rotation 
% Correct order for inverse: inv(T_total) = inv(T_manual) * inv(T_align) 
tform_back.T = invert(tform_manual).T * tform_back.T; 
 
figure(2) 
pcshow(ptCloud_rotated); 
title('Automatically Rotated DTM'); 
xlabel('X-axis'); 
ylabel('Y-axis'); 
zlabel('Z-axis'); 
 
%% 
%downsample the ROTATED point cloud 
if isfield(params, 'enableDownsampling') && params.enableDownsampling 
    ptCloud_down = pcdownsample(ptCloud_rotated,'gridAverage',gridStep); 
    ptCloudWithGround_Out = pcdenoise(ptCloud_down); 
    ptCloudWithGround_Out = removeInvalidPoints(ptCloudWithGround_Out); 
    figure(3) 
    pcshow(ptCloudWithGround_Out.Location) 
    title("Downsampled Rotated Ground Points") 
else 
    % Skip downsampling and denoising, but ensure invalid points are removed 
    ptCloudWithGround_Out = removeInvalidPoints(ptCloud_rotated); 
    figure(3) 
    pcshow(ptCloudWithGround_Out.Location) 
    title("Rotated Ground Points (Original Resolution)") 
end 
 
%% 
%procedure for create .stl file 
x = ptCloudWithGround_Out.Location(:,1); 
y = ptCloudWithGround_Out.Location(:,2); 
z = ptCloudWithGround_Out.Location(:,3); 
 
%% 
% Identify vertices to keep (below cutoff) AND ensure all values are finite 
valid_idx = isfinite(x) & isfinite(y) & isfinite(z); 
 
% Filter vertices 
x_filtered = x(valid_idx); 
y_filtered = y(valid_idx); 
z_filtered = z(valid_idx); 
 
% Remove duplicates in XY before triangulation 
[xy_unique, ~, ic] = unique([x_filtered(:), y_filtered(:)], 'rows', 'stable'); 
% For duplicate XY points, take the mean Z value 
z_unique = accumarray(ic, z_filtered, [], @mean); 
 
% Update filtered arrays 
x_filtered = xy_unique(:, 1); 
y_filtered = xy_unique(:, 2); 
z_filtered = z_unique; 
 
fprintf('Points after deduplication: %d\n', length(x_filtered)); 
 
% Now create triangulation - points will be in same order 
dt_filtered = delaunayTriangulation(x_filtered, y_filtered); 
faces = dt_filtered.ConnectivityList; 
 
% Create vertices directly - no need for ismember 
vertices_top_o = [x_filtered, y_filtered, z_filtered]; 
vertices_top = vertices_top_o; 
 
faces_t_o = filter_by_length(faces, vertices_top_o(:,1), vertices_top_o(:,2), max_edge_length); 
 
%% HERE CHECK THE WELDING ANGLE 
if params.enableWelding 
    weldingAngle = params.weldingAngle; 
    peakThreshold = params.peakThreshold; 
    coplanarAngle = 60; 
    maxIterations = 10; 
    ncycle = 3; 
 
    n = 1;      
    faces = faces_t_o;      
    while n <= ncycle      
        [vertices_top, faces, modifiedIndices, stats] = advancedMeshWelding(...      
                 vertices_top, faces, weldingAngle, coplanarAngle, maxIterations, peakThreshold);      
        n = n+1;      
    end    
else 
    faces = faces_t_o; 
end 
 
% Flat bottom surface 
z_flat = min(z_filtered) - z_deep; 
x_filtered = vertices_top(:,1); 
y_filtered = vertices_top(:,2); 
z_filtered = vertices_top(:,3); 
 
if params.enableInterpolation 
    % Create scattered interpolant 
    F = scatteredInterpolant(vertices_top(:,1), vertices_top(:,2), vertices_top(:,3), interpolation, extrapolation); 
 
    x_array = min(x_filtered):gridStep:max(x_filtered);      
    y_array = min(y_filtered):gridStep:max(y_filtered);      
      
    [Xq, Yq] = meshgrid(x_array, y_array);      
    Zq = F(Xq, Yq);      
      
    x_array_fine = min(x_filtered):gridStep/2:max(x_filtered);      
    y_array_fine = min(y_filtered):gridStep/2:max(y_filtered);      
    [Xq_c, Yq_c] = meshgrid(x_array_fine, y_array_fine);      
      
    valid = ~isnan(Zq(:));      
    if sum(valid) < 10      
        error('Insufficient valid data points for interpolation');      
    end      
      
    F_cubic = scatteredInterpolant(Xq(valid), Yq(valid), Zq(valid), interpolation, extrapolation);      
    Zq_c = F_cubic(Xq_c, Yq_c);      
      
    x_filtered = Xq_c(:);      
    y_filtered = Yq_c(:);      
    z_filtered = Zq_c(:);      
      
    validIdx = isfinite(z_filtered) & isfinite(x_filtered) & isfinite(y_filtered);      
      
    fprintf('Total points: %d\n', length(z_filtered));      
    fprintf('Valid finite points: %d\n', sum(validIdx));      
    fprintf('Invalid points removed: %d\n', sum(~validIdx));      
      
    if sum(validIdx) < 3      
        error('Insufficient valid points after filtering. Check your input data coverage.');      
    end      
      
    x_filtered = x_filtered(validIdx);      
    y_filtered = y_filtered(validIdx);      
    z_filtered = z_filtered(validIdx);    
end 
 
% Remove duplicates in XY before second triangulation 
[xy_unique2, ~, ic2] = unique([x_filtered(:), y_filtered(:)], 'rows', 'stable'); 
% For duplicate XY points, take the mean Z value 
z_unique2 = accumarray(ic2, z_filtered, [], @mean); 
 
% Update filtered arrays 
x_filtered = xy_unique2(:, 1); 
y_filtered = xy_unique2(:, 2); 
z_filtered = z_unique2; 
 
fprintf('Points after second deduplication: %d\n', length(x_filtered)); 
 
% Create Delaunay triangulation 
DT = delaunayTriangulation(x_filtered, y_filtered); 
 
% Create vertices directly - no need for ismember 
vertices_top = [x_filtered, y_filtered, z_filtered]; 
 
% Create triangulation mesh 
faces = triangulation(DT.ConnectivityList, vertices_top); 
 
faces_t = filter_by_length(faces.ConnectivityList, vertices_top(:,1), vertices_top(:,2), max_edge_length); 
% Now proceed with the 3D object 
vertices_bottom = [vertices_top(:,1), vertices_top(:,2), z_flat * ones(size(vertices_top, 1), 1)]; 
dt = delaunayTriangulation(vertices_top(:,1), vertices_top(:,2)); 
T_filtered = dt.ConnectivityList; 
 
T_filtered = filter_by_length(T_filtered, vertices_top(:,1), vertices_top(:,2), max_edge_length); 
 
% Combine all vertices 
vertices_all = [vertices_top; vertices_bottom]; 
n = size(vertices_top, 1); 
 
% Create bottom faces (reverse to orient normals downward) 
faces_bottom = fliplr(T_filtered + n); 
 
% Side walls 
dt_filtered_faces = triangulation(T_filtered, vertices_top(:,1), vertices_top(:,2)); 
boundary_edges = freeBoundary(dt_filtered_faces); 
 
num_edges = size(boundary_edges, 1); 
faces_side = zeros(2 * num_edges, 3); 
 
for i = 1:num_edges 
    v1 = boundary_edges(i, 1); 
    v2 = boundary_edges(i, 2); 
    v1b = v1 + n; 
    v2b = v2 + n; 
 
    faces_side(2*i - 1, :) = [v1, v1b, v2b];      
    faces_side(2*i, :) = [v1, v2b, v2];      
end 
 
% Combine all faces 
faces_all = [faces_t; faces_bottom; faces_side]; 
 
% Create final triangulation object 
TR = triangulation(faces_all, vertices_all); 
 
%% AUTOMATICALLY ROTATE MESH BACK TO ORIGINAL POSITION 
% Determine which inverse rotations to apply
TR_rotated_points = TR.Points;

% 1. Inverse Manual Rotation
if isfield(params, 'skipManualInverse') && params.skipManualInverse
    fprintf('Skipping Manual Inverse Rotation.\n');
else
    % Apply inverse manual rotation
    TR_rotated_points = transformPointsForward(invert(tform_manual), TR_rotated_points);
end

% 2. Inverse Automatic Alignment
if isfield(params, 'skipAutoInverse') && params.skipAutoInverse
    fprintf('Skipping Automatic Inverse Rotation (Alignment).\n');
else
    % Apply inverse alignment rotation
    TR_rotated_points = transformPointsForward(invert(tform_align), TR_rotated_points);
end

% Create the final, correctly oriented triangulation 
TR_rotated = triangulation(TR.ConnectivityList, TR_rotated_points); 

% Export rotated STL 
stlwrite(TR_rotated, stl_out); 
 
% Visualize the result 
figure(4) 
subplot(1,2,1); 
trisurf(faces_t_o, vertices_top_o(:,1), vertices_top_o(:,2), vertices_top_o(:,3), 'FaceAlpha', 0.8); 
title('Original Mesh'); 
axis equal; grid on; 
 
subplot(1,2,2); 
trisurf(faces_t, vertices_top(:,1), vertices_top(:,2), vertices_top(:,3), 'FaceAlpha', 0.8); 
title('After Welding'); 
axis equal; grid on; 
 
figure(5) 
trisurf(TR_rotated, 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k'); 
axis equal 
xlabel('X'); ylabel('Y'); zlabel('Z'); 
title('Solid Mesh with Edge Length Limit'); 
view(3); 
 
%% INTERACTIVE PLANE SELECTOR FUNCTION 
function [planePosition, planeNormal, finalDistance, targetAxis] = interactivePlaneSelector(x, y, z, initialDistance) 
% Create figure for interactive plane selection 
fig = figure('Name', 'Interactive Plane Selector', 'NumberTitle', 'off', ... 
'Position', [100 100 1200 800]); % Increased height for new controls 
 
% Create main axes for 3D view     
ax = axes('Parent', fig, 'Position', [0.05 0.15 0.7 0.8]);     
 
% Downsample points for visualization     
step = max(1, floor(length(x)/50000));     
x_plot = x(1:step:end);     
y_plot = y(1:step:end);     
z_plot = z(1:step:end);     
 
% Plot point cloud     
scatter3(ax, x_plot, y_plot, z_plot, 0.5, z_plot, 'filled');     
colormap(ax, 'parula');     
hold(ax, 'on');     
axis(ax, 'equal');     
grid(ax, 'on');     
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');     
title(ax, 'Click and drag arrows to move plane, use sliders to adjust');     
view(ax, 3);     
rotate3d(ax, 'on');     
 
% Initialize plane parameters     
centroid = [mean(x), mean(y), mean(z)];     
planePos = centroid;     
rotationAngle = 0; % Rotation around Z-axis in degrees     
planeDistance = initialDistance; % Editable distance     
targetAxis = 'Minimal'; % Default vertical alignment axis    
 
% Calculate plane size for visualization     
rangeX = max(x) - min(x);     
rangeY = max(y) - min(y);     
planeSize = max(rangeX, rangeY) * 0.8;     
 
% --- Create UI controls ---     
panel_pos_y_start = 750;     
 
% Rotation control     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start 200 20], ...     
          'String', 'Plane Rotation (Z-axis)', 'FontSize', 10, 'FontWeight', 'bold');     
rotSlider = uicontrol('Style', 'slider', 'Position', [850 panel_pos_y_start-30 200 20], ...     
                      'Min', -180, 'Max', 180, 'Value', 0, ...     
                      'Callback', @updatePlaneRotation);     
rotText = uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-55 200 20], ...     
                    'String', sprintf('Angle: %.1f°', 0), 'FontSize', 9);     
 
% Distance control     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-90 200 20], ...     
          'String', 'Distance Between Planes (in meters)', 'FontSize', 10, 'FontWeight', 'bold');     
maxRange = max(rangeX, rangeY);     
distSlider = uicontrol('Style', 'slider', 'Position', [850 panel_pos_y_start-120 200 20], ...     
                      'Min', 1, 'Max', maxRange, 'Value', planeDistance, ...     
                      'Callback', @updatePlaneDistance);     
distText = uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-145 200 20], ...     
                    'String', sprintf('Distance: %.1f m', planeDistance), 'FontSize', 9);     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-175 100 20], ...     
          'String', 'Precise value:', 'FontSize', 9, 'HorizontalAlignment', 'left');     
distEdit = uicontrol('Style', 'edit', 'Position', [950 panel_pos_y_start-175 100 25], ...     
                     'String', sprintf('%.1f', planeDistance), 'FontSize', 9, ...     
                     'Callback', @updateDistanceFromEdit);     
 
% Plane Position controls (NEW)     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-225 200 20], ...     
          'String', 'Plane Position (Center)', 'FontSize', 10, 'FontWeight', 'bold');     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-255 20 20], 'String', 'X:', 'FontSize', 9, 'HorizontalAlignment', 'left');     
posEditX = uicontrol('Style', 'edit', 'Position', [875 panel_pos_y_start-255 100 25], ...     
                     'String', sprintf('%.2f', planePos(1)), 'FontSize', 9, 'Callback', @updatePositionFromEdit);     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-285 20 20], 'String', 'Y:', 'FontSize', 9, 'HorizontalAlignment', 'left');     
posEditY = uicontrol('Style', 'edit', 'Position', [875 panel_pos_y_start-285 100 25], ...     
                     'String', sprintf('%.2f', planePos(2)), 'FontSize', 9, 'Callback', @updatePositionFromEdit);     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-315 20 20], 'String', 'Z:', 'FontSize', 9, 'HorizontalAlignment', 'left');     
posEditZ = uicontrol('Style', 'edit', 'Position', [875 panel_pos_y_start-315 100 25], ...     
                     'String', sprintf('%.2f', planePos(3)), 'FontSize', 9, 'Callback', @updatePositionFromEdit);     
 
% Alignment Axis Selection    
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-370 200 20], ...     
          'String', 'Align Vertical With:', 'FontSize', 10, 'FontWeight', 'bold');    
% Using 'pixels' for better control of layout  
axisGroup = uibuttongroup('Parent', fig, 'Units', 'pixels', 'Position', [850 panel_pos_y_start-415 250 40], ...    
                          'BorderType', 'none', 'SelectionChangedFcn', @updateAxisSelection);    
uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'Y-axis', 'Position', [10 5 60 25]);    
uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'X-axis', 'Position', [70 5 60 25]);    
uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'Minimal', 'Position', [130 5 70 25], 'Value', 1);    
 
% Point count display     
pointCountText = uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-465 200 40], ...     
                           'String', '', 'FontSize', 9, 'HorizontalAlignment', 'left');     
 
% Confirm button     
uicontrol('Style', 'pushbutton', 'Position', [850 panel_pos_y_start-525 200 40], ...     
          'String', 'Confirm Selection', 'FontSize', 12, 'Callback', @confirmSelection);     
 
% Instructions     
uicontrol('Style', 'text', 'Position', [850 panel_pos_y_start-615 200 80], ...     
          'String', 'Instructions: Drag arrows to move plane. Use sliders or enter values to adjust rotation, distance, and position. Select alignment axis.', ...     
          'FontSize', 9, 'HorizontalAlignment', 'left');     
 
% Storage for graphics objects     
planeHandles = struct();     
arrowHandles = struct();     
 
% Draw initial planes     
updatePlaneVisualization();     
 
% Wait for user confirmation     
uiwait(fig);     
 
% Return final values     
finalDistance = planeDistance;     
planePosition = planePos;     
planeNormal = calculatePlaneNormal();     
% targetAxis is updated via callback    
 
% --- Nested Functions ---     
    function updateAxisSelection(~, event)    
        selected = get(event.NewValue, 'String');    
        if strcmp(selected, 'X-axis')    
            targetAxis = 'X';    
        elseif strcmp(selected, 'Y-axis')    
            targetAxis = 'Y';    
        else    
            targetAxis = 'Minimal';    
        end    
    end    
 
    function normal = calculatePlaneNormal()     
        angleRad = deg2rad(rotationAngle);     
        normal = [sin(angleRad), cos(angleRad), 0];     
        if norm(normal) > 0, normal = normal / norm(normal); end     
    end     
 
    function updatePlaneRotation(src, ~)     
        rotationAngle = get(src, 'Value');     
        set(rotText, 'String', sprintf('Angle: %.1f°', rotationAngle));     
        updatePlaneVisualization();     
    end     
 
    function updatePlaneDistance(src, ~)     
        planeDistance = get(src, 'Value');     
        set(distText, 'String', sprintf('Distance: %.1f m', planeDistance));     
        set(distEdit, 'String', sprintf('%.1f', planeDistance));     
        updatePlaneVisualization();     
    end     
 
    function updateDistanceFromEdit(src, ~)     
        newDist = str2double(get(src, 'String'));     
        if ~isnan(newDist) && newDist > 0     
            planeDistance = newDist;     
            sliderMax = get(distSlider, 'Max');     
            if newDist > sliderMax, set(distSlider, 'Max', newDist * 1.2); end     
            set(distSlider, 'Value', newDist);     
            set(distText, 'String', sprintf('Distance: %.1f m', planeDistance));     
            updatePlaneVisualization();     
        else     
            set(src, 'String', sprintf('%.1f', planeDistance)); % Reset if invalid     
        end     
    end     
 
    function updatePositionFromEdit(~, ~)     
        newX = str2double(get(posEditX, 'String'));     
        newY = str2double(get(posEditY, 'String'));     
        newZ = str2double(get(posEditZ, 'String'));     
        if ~isnan(newX), planePos(1) = newX; end     
        if ~isnan(newY), planePos(2) = newY; end     
        if ~isnan(newZ), planePos(3) = newZ; end     
        updatePlaneVisualization();     
    end     
 
    function updatePlaneVisualization()     
        % Delete old graphics     
        if isfield(planeHandles, 'plane1') && isvalid(planeHandles.plane1), delete(planeHandles.plane1); end     
        if isfield(planeHandles, 'plane2') && isvalid(planeHandles.plane2), delete(planeHandles.plane2); end     
        if isfield(arrowHandles, 'arrowX') && isvalid(arrowHandles.arrowX), delete(arrowHandles.arrowX); end     
        if isfield(arrowHandles, 'arrowY') && isvalid(arrowHandles.arrowY), delete(arrowHandles.arrowY); end     
        if isfield(arrowHandles, 'arrowZ') && isvalid(arrowHandles.arrowZ), delete(arrowHandles.arrowZ); end     
 
        % Update position edit boxes     
        set(posEditX, 'String', sprintf('%.2f', planePos(1)));     
        set(posEditY, 'String', sprintf('%.2f', planePos(2)));     
        set(posEditZ, 'String', sprintf('%.2f', planePos(3)));     
 
        % Get current plane normal     
        normal = calculatePlaneNormal();     
 
        % Calculate and display number of points between planes     
        planePosExpanded = repmat(planePos, length(x), 1);     
        normalExpanded = repmat(normal, length(x), 1);     
        d1 = dot([x, y, z] - planePosExpanded, normalExpanded, 2);     
        d2 = dot([x, y, z] - (planePosExpanded + planeDistance * normalExpanded), normalExpanded, 2);     
        numPoints = sum((d1 >= 0) & (d2 <= 0));     
        set(pointCountText, 'String', sprintf('Points selected: %d\nTotal points: %d\nPercentage: %.1f%%', ...     
            numPoints, length(x), 100*numPoints/length(x)));     
 
        % Create perpendicular vectors for plane visualization     
        if abs(normal(3)) < 0.9, v1 = cross(normal, [0 0 1]); else, v1 = cross(normal, [1 0 0]); end     
        v1 = v1 / norm(v1);     
        v2 = cross(normal, v1);     
        v2 = v2 / norm(v2);     
 
        % Create and draw planes     
        [u, v] = meshgrid(linspace(-planeSize/2, planeSize/2, 10));     
        planePts1 = planePos + u(:)*v1 + v(:)*v2;     
        planeHandles.plane1 = surf(ax, reshape(planePts1(:,1), size(u)), reshape(planePts1(:,2), size(u)), reshape(planePts1(:,3), size(u)), ...     
                                   'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');     
        planePos2 = planePos + planeDistance * normal;     
        planePts2 = planePos2 + u(:)*v1 + v(:)*v2;     
        planeHandles.plane2 = surf(ax, reshape(planePts2(:,1), size(u)), reshape(planePts2(:,2), size(u)), reshape(planePts2(:,3), size(u)), ...     
                                   'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');     
 
        % Draw interactive arrows     
        arrowLength = planeSize * 0.3;     
        arrowHandles.arrowX = drawInteractiveArrow(planePos, [1 0 0], arrowLength, 'r', @moveX);     
        arrowHandles.arrowY = drawInteractiveArrow(planePos, [0 1 0], arrowLength, 'g', @moveY);     
        arrowHandles.arrowZ = drawInteractiveArrow(planePos, [0 0 1], arrowLength, 'b', @moveZ);     
    end     
 
    function h = drawInteractiveArrow(startPos, direction, length, color, callback)     
        endPos = startPos + direction * length;     
        h = quiver3(ax, startPos(1), startPos(2), startPos(3), ...     
                    direction(1)*length, direction(2)*length, direction(3)*length, ...     
                    'Color', color, 'LineWidth', 3, 'MaxHeadSize', 0.5, ...     
                    'ButtonDownFcn', @(src,evt) startDrag(callback));     
    end     
 
    function startDrag(moveCallback)     
        set(fig, 'WindowButtonMotionFcn', moveCallback);     
        set(fig, 'WindowButtonUpFcn', @stopDrag);     
    end     
 
    function stopDrag(~, ~)     
        set(fig, 'WindowButtonMotionFcn', '');     
        set(fig, 'WindowButtonUpFcn', '');     
    end     
 
    function moveX(~, ~), pt = get(ax, 'CurrentPoint'); planePos(1) = pt(1,1); updatePlaneVisualization(); end     
    function moveY(~, ~), pt = get(ax, 'CurrentPoint'); planePos(2) = pt(1,2); updatePlaneVisualization(); end     
    function moveZ(~, ~), pt = get(ax, 'CurrentPoint'); planePos(3) = pt(1,3); updatePlaneVisualization(); end     
 
    function confirmSelection(~, ~)     
        % No need to recalculate, values are stored in the outer scope     
        uiresume(fig);     
        close(fig);     
    end     
end 
 
%% TEST WELDING ANGLE 
 
function [faces_t] = filter_by_length(T_filtered,x_filtered, y_filtered, max_edge_length) 
% Filter triangles by edge length 
num_triangles = size(T_filtered, 1); 
valid_triangles = true(num_triangles, 1); 
 
for i = 1:num_triangles      
    % Get vertices of this triangle      
    v1 = [x_filtered(T_filtered(i,1)), y_filtered(T_filtered(i,1))];      
    v2 = [x_filtered(T_filtered(i,2)), y_filtered(T_filtered(i,2))];      
    v3 = [x_filtered(T_filtered(i,3)), y_filtered(T_filtered(i,3))];      
          
    % Calculate edge lengths      
    edge1 = norm(v2 - v1);      
    edge2 = norm(v3 - v2);      
    edge3 = norm(v1 - v3);      
          
    % Check if any edge exceeds the limit      
    if max([edge1, edge2, edge3]) > max_edge_length      
        valid_triangles(i) = false;      
    end      
end      
% Keep only valid triangles      
faces_t = T_filtered(valid_triangles, :);      
end 
 
function tform_manual = interactiveRotationSelector(ptCloud) 
% Create a figure for interactive rotation selection 
fig = figure('Name', 'Interactive Rotation Selector', 'NumberTitle', 'off', ... 
'Position', [200 200 1000 700]); 
 
% Create main axes for 3D view     
ax = axes('Parent', fig, 'Position', [0.05 0.1 0.65 0.85]);     
pcshow(ptCloud);     
hold(ax, 'on');     
axis(ax, 'equal');     
grid(ax, 'on');     
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');     
title(ax, 'Define a new rotation axis and angle');     
view(ax, 3);     
rotate3d(ax, 'on');     
 
% Initial rotation parameters     
rotationAxis = [0 0 1]; % Default axis     
rotationAngle = 0;      % Default angle in degrees     
tform_manual = affinetform3d(eye(4)); % Default transform (identity)     
 
% --- UI Controls ---     
panel_pos_y_start = 650;     
 
% Rotation Axis controls (Radio Buttons)     
axisGroup = uibuttongroup('Parent', fig, 'Title', 'Rotation Axis', 'FontSize', 10, ...     
                          'FontWeight', 'bold', 'Position', [0.75 0.7 0.2 0.25], ...     
                          'SelectionChangedFcn', @updateVisualization);     
uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'X-axis', 'Position', [10 100 100 25]);     
uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'Y-axis', 'Position', [10 65 100 25]);     
uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'Z-axis', 'Position', [10 30 100 25], 'Value', 1);     
 
% Rotation Angle controls     
uicontrol('Style', 'text', 'Position', [750 panel_pos_y_start-140 200 20], ...     
          'String', 'Rotation Angle (degrees)', 'FontSize', 10, 'FontWeight', 'bold');     
angleSlider = uicontrol('Style', 'slider', 'Position', [750 panel_pos_y_start-170 200 20], ...     
                        'Min', -180, 'Max', 180, 'Value', rotationAngle, ...     
                        'Callback', @updateAngleFromSlider);     
angleEdit = uicontrol('Style', 'edit', 'Position', [750 panel_pos_y_start-200 100 25], ...     
                      'String', num2str(rotationAngle), 'Callback', @updateAngleFromEdit);     
 
% Translation Checkbox     
translateCheckbox = uicontrol('Style', 'checkbox', 'Position', [750 panel_pos_y_start-250 200 25], ...     
                              'String', 'Translate lowest point to XY plane', 'FontSize', 9);     
 
% Confirm button     
uicontrol('Style', 'pushbutton', 'Position', [750 panel_pos_y_start-300 200 40], ...     
          'String', 'Confirm Rotation', 'FontSize', 12, 'Callback', @confirmSelection);     
 
% Storage for graphics objects     
axisHandle = [];     
 
% Initial visualization     
updateVisualization();     
 
% Wait for user to confirm     
uiwait(fig);     
 
% --- Nested Functions ---     
    function updateAngleFromSlider(src, ~)     
        rotationAngle = get(src, 'Value');     
        set(angleEdit, 'String', sprintf('%.1f', rotationAngle));     
        updateVisualization();     
    end     
 
    function updateAngleFromEdit(src, ~)     
        val = str2double(get(src, 'String'));     
        if ~isnan(val)     
            rotationAngle = val;     
            % Clamp slider value     
            if val < get(angleSlider, 'Min'), val = get(angleSlider, 'Min'); end     
            if val > get(angleSlider, 'Max'), val = get(angleSlider, 'Max'); end     
            set(angleSlider, 'Value', val);     
        else     
            set(src, 'String', num2str(rotationAngle)); % Reset if invalid     
        end     
        updateVisualization();     
    end     
 
    function updateVisualization(~, ~)     
        % Delete old axis visualization     
        if ~isempty(axisHandle) && isvalid(axisHandle), delete(axisHandle); end     
 
        % Determine selected axis     
        selectedButton = get(axisGroup, 'SelectedObject');     
        axisString = get(selectedButton, 'String');     
        switch axisString     
            case 'X-axis'     
                rotationAxis = [1 0 0];     
            case 'Y-axis'     
                rotationAxis = [0 1 0];     
            case 'Z-axis'     
                rotationAxis = [0 0 1];     
        end     
 
        % Visualize the axis vector     
        centroid = mean(ptCloud.Location, 1);     
        axisLength = max(ptCloud.XLimits) - min(ptCloud.XLimits);     
        axisEnd = centroid + rotationAxis * axisLength * 0.5;     
 
        axisHandle = quiver3(ax, centroid(1), centroid(2), centroid(3), ...     
                             (axisEnd(1)-centroid(1)), (axisEnd(2)-centroid(2)), (axisEnd(3)-centroid(3)), ...     
                             'Color', 'magenta', 'LineWidth', 3, 'MaxHeadSize', 0.3, 'AutoScale', 'off');     
    end     
 
    function confirmSelection(~, ~)     
        % Get final rotation axis     
        selectedButton = get(axisGroup, 'SelectedObject');     
        axisString = get(selectedButton, 'String');     
        switch axisString     
            case 'X-axis', rotationAxis = [1 0 0];     
            case 'Y-axis', rotationAxis = [0 1 0];     
            case 'Z-axis', rotationAxis = [0 0 1];     
        end     
             
        % Get final rotation angle     
        rotationAngle = str2double(get(angleEdit, 'String'));     
             
        % Create rotation matrix     
        R_manual = axang2rotm([rotationAxis deg2rad(rotationAngle)]);     
        affine_matrix_manual = eye(4);     
        affine_matrix_manual(1:3, 1:3) = R_manual'; % Transpose for post-multiplication     
             
        % Check if translation is needed     
        if get(translateCheckbox, 'Value') == 1     
            % Create a temporary transform for rotation only     
            tform_rot_only = affinetform3d(affine_matrix_manual);     
            % Rotate the point cloud to find its new minimum Z     
            rotated_pts = transformPointsForward(tform_rot_only, ptCloud.Location);     
            minZ = min(rotated_pts(:,3));     
            % Add translation to the matrix     
            translation_vec = [0, 0, -minZ];     
            affine_matrix_manual(4, 1:3) = translation_vec;     
        end     
             
        % Create the final combined transformation     
        tform_manual = affinetform3d(affine_matrix_manual);     
             
        uiresume(fig);     
        close(fig);     
    end     
end 
 
function [newVertices, newFaces, spikeVertices] = filterMeshPeaks(vertices, faces, spikeAngle, heightThreshold) 
% FILTERMESHPEAKS Remove peak- or pyramid-like protrusions from a 3D mesh 
% 
% Inputs: 
% vertices - Nx3 array of vertex coordinates 
% faces - Mx3 array of face indices (1-based) 
% spikeAngle - Dihedral threshold in degrees (e.g., 45) 
% heightThreshold - Max allowed deviation from local plane (e.g., 0.005) 
% 
% Outputs: 
% newVertices - Cleaned vertex coordinates 
% newFaces - Cleaned face connectivity 
% spikeVertices - Indices of removed spike vertices 
 
if nargin < 3, spikeAngle = 45; end      
if nargin < 4, heightThreshold = 0.01; end      
  
nV = size(vertices,1);      
nF = size(faces,1);      
  
%% --- Compute face normals ---      
p1 = vertices(faces(:,1),:);      
p2 = vertices(faces(:,2),:);      
p3 = vertices(faces(:,3),:);      
fn = cross(p2 - p1, p3 - p1, 2);      
fn = fn ./ max(vecnorm(fn,2,2), eps);  % normalize      
  
%% --- Build vertex-face adjacency ---      
vertexFaces = cell(nV,1);      
for f = 1:nF      
    for j = 1:3      
        vertexFaces{faces(f,j)}(end+1) = f;      
    end      
end      
  
%% --- Compute local curvature (angle deviation + height) ---      
spikeMask = false(nV,1);      
for v = 1:nV      
    fIdx = vertexFaces{v};      
    if numel(fIdx) < 3      
        spikeMask(v) = true; % tip or border      
        continue;      
    end      
    localNormals = fn(fIdx,:);      
    avgNormal = mean(localNormals,1);      
    avgNormal = avgNormal ./ norm(avgNormal);      
    % angle deviation (curvature proxy)      
    cosang = sum(localNormals .* avgNormal, 2);      
    cosang = min(1,max(-1,cosang));      
    ang = acosd(cosang);      
    maxAngle = max(ang);      
    if maxAngle > spikeAngle      
        % height deviation test      
        localVerts = unique(faces(fIdx,:));      
        pts = vertices(localVerts,:);      
        centroid = mean(pts,1);      
        d = dot(vertices(v,:) - centroid, avgNormal);      
        if abs(d) > heightThreshold      
            spikeMask(v) = true;      
        end      
    end      
end      
  
%% --- Remove faces that include spike vertices ---      
spikeVertices = find(spikeMask);      
keepFace = ~any(ismember(faces, spikeVertices), 2);      
newFaces = faces(keepFace,:);      
  
%% --- Remove unused vertices and reindex ---      
if isempty(newFaces)     
    newVertices = [];     
    spikeVertices = find(spikeMask);     
    return;     
end     
[usedVerts, ~, newFaceIndices] = unique(newFaces(:));     
newVertices = vertices(usedVerts, :);     
newFaces = reshape(newFaceIndices, [], size(newFaces, 2));     
end 
 
function [newVertices, faces, modifiedIndices, stats] = advancedMeshWelding(vertices, faces, weldingAngle, coplanarAngle, maxIterations, peakThreshold) 
% ADVANCEDMESHWELDING Filter vertices using interior and dihedral angles 
% 
% Input: 
% vertices - Nx3 matrix of vertex coordinates [x, y, z] 
% faces - Mx3 matrix of face indices (triangular mesh) 
% weldingAngle - Threshold for interior angle sum (e.g., 30°) 
% coplanarAngle - Threshold for dihedral angles (e.g., 175°) 
% Vertices with dihedral angles > coplanarAngle are near-coplanar 
% maxIterations - Maximum iterations for coplanar fixing (default: 10) 
% peakThreshold - Max allowed deviation from local plane (e.g., 0.005) 
% 
% Output: 
% newVertices - Modified Nx3 vertex matrix 
% modifiedIndices - Structure with interior and dihedral modifications 
% stats - Statistics about the welding process 
 
if nargin < 5      
    maxIterations = 10;      
end    
if nargin < 6    
    peakThreshold = 0.005;    
end      
      
newVertices = vertices;      
modifiedIndices.interior = [];      
modifiedIndices.dihedral = [];      
stats.interiorFixed = 0;      
stats.dihedralIterations = 0;      
    
% Check if faces are valid before processing    
if isempty(faces) || max(faces(:)) > size(newVertices, 1)    
    warning('Invalid face indices detected. Skipping mesh welding.');    
    return;    
end    
      
% Step 1: Fix narrow interior angles      
[newVertices, faces, removed] = filterMeshPeaks(newVertices, faces, weldingAngle, peakThreshold);      
    
% Check again after filterMeshPeaks    
if isempty(faces) || isempty(newVertices)    
    warning('No faces remaining after filterMeshPeaks. Returning empty mesh.');    
    return;    
end    
      
% Step 2: Iteratively fix near-coplanar dihedral angles      
coplanarVertices = [];      
      
% Check all vertices for near-coplanar conditions      
for vertexIdx = 1:size(newVertices, 1)      
    [maxDihedral, avgDihedral, dihedralAngles] = calculateDihedralAngles(newVertices, faces, vertexIdx);      
          
    % Check if any dihedral angle exceeds threshold (near-coplanar)      
    if maxDihedral > coplanarAngle      
        coplanarVertices = [coplanarVertices; vertexIdx];      
    end      
end      
      
% Fix all coplanar vertices in this iteration      
for i = 1:length(coplanarVertices)      
    vertexIdx = coplanarVertices(i);      
    neighborVertices = getNeighborVertices(newVertices, faces, vertexIdx);      
          
    if size(neighborVertices, 1) >= 3      
        oldPosition = newVertices(vertexIdx, :);      
        newPosition = fitPlaneAndProject(oldPosition, neighborVertices);      
        displacement = norm(newPosition - oldPosition);      
              
        newVertices(vertexIdx, :) = newPosition;      
              
        % Track if this is a new modification      
        if ~ismember(vertexIdx, modifiedIndices.dihedral)      
            modifiedIndices.dihedral = [modifiedIndices.dihedral; vertexIdx];      
        end      
    end      
end      
  
% Check and delete the vertex with only one face connected (isolated peaks)      
vertexFaceCount = accumarray(faces(:), 1, [size(newVertices,1), 1]);      
      
% Identify vertices used in only one face      
singleFaceVerts = find(vertexFaceCount == 1);      
      
% Remove any faces that use those vertices      
facesToDelete = any(ismember(faces, singleFaceVerts), 2);      
faces(facesToDelete, :) = [];      
      
% Remove unused vertices and reindex faces      
if isempty(faces)    
    % If no faces left, return empty arrays    
    newVertices = [];    
    faces = [];    
    return;    
end    
    
[usedVerts, ~, newFaceIndices] = unique(faces(:));     
newVertices = newVertices(usedVerts, :);     
faces = reshape(newFaceIndices, [], size(faces, 2));     
end 
 
function [totalAngle, individualAngles] = calculateInteriorAngle(vertices, faces, vertexIdx) 
% Calculate sum of interior angles at a vertex (within each triangle face) 
 
[faceRows, ~] = find(faces == vertexIdx);      
      
if isempty(faceRows)      
    totalAngle = 0;      
    individualAngles = [];      
    return;      
end      
      
centerVertex = vertices(vertexIdx, :);      
individualAngles = [];      
      
for i = 1:length(faceRows)      
    face = faces(faceRows(i), :);      
    otherVertices = face(face ~= vertexIdx);      
          
    if length(otherVertices) ~= 2      
        continue;      
    end      
          
    v1 = vertices(otherVertices(1), :);      
    v2 = vertices(otherVertices(2), :);      
          
    vec1 = v1 - centerVertex;      
    vec2 = v2 - centerVertex;      
          
    cosAngle = dot(vec1, vec2) / (norm(vec1) * norm(vec2));      
    cosAngle = max(-1, min(1, cosAngle));      
    angle = acosd(cosAngle);      
          
    individualAngles = [individualAngles; angle];      
end      
      
totalAngle = sum(individualAngles);      
end 
 
function [maxDihedral, avgDihedral, dihedralAngles] = calculateDihedralAngles(vertices, faces, vertexIdx) 
% Calculate dihedral angles between adjacent faces at a vertex 
 
[faceRows, ~] = find(faces == vertexIdx);      
      
if length(faceRows) < 2      
    maxDihedral = 0;      
    avgDihedral = 0;      
    dihedralAngles = [];      
    return;      
end      
      
dihedralAngles = [];      
      
% For each pair of faces sharing this vertex      
for i = 1:length(faceRows)      
    for j = i+1:length(faceRows)      
        face1 = faces(faceRows(i), :);      
        face2 = faces(faceRows(j), :);      
              
        % Check if faces share an edge (have 2 common vertices)      
        commonVerts = intersect(face1, face2);      
        if length(commonVerts) == 2      
            % Calculate normal vectors for both faces      
            normal1 = calculateFaceNormal(vertices, face1);      
            normal2 = calculateFaceNormal(vertices, face2);      
                  
            % Dihedral angle between normals      
            cosAngle = dot(normal1, normal2);      
            cosAngle = max(-1, min(1, cosAngle));      
            angle = acosd(cosAngle);      
                  
            % Convert to dihedral angle (0° = folded, 180° = coplanar)      
            dihedralAngle = 180-angle;      
                  
            dihedralAngles = [dihedralAngles; dihedralAngle];      
        end      
    end      
end      
      
if isempty(dihedralAngles)      
    maxDihedral = 0;      
    avgDihedral = 0;      
else      
    maxDihedral = max(dihedralAngles);      
    avgDihedral = mean(dihedralAngles);      
end      
end 
 
function normal = calculateFaceNormal(vertices, face) 
% Calculate normal vector of a triangular face 
 
v1 = vertices(face(1), :);      
v2 = vertices(face(2), :);      
v3 = vertices(face(3), :);      
      
edge1 = v2 - v1;      
edge2 = v3 - v1;      
      
normal = cross(edge1, edge2);      
normal = normal / norm(normal);      
end 
 
function neighborVertices = getNeighborVertices(vertices, faces, vertexIdx) 
% Get all vertices connected to the specified vertex 
 
[faceRows, ~] = find(faces == vertexIdx);      
      
neighborIndices = [];      
for i = 1:length(faceRows)      
    face = faces(faceRows(i), :);      
    neighbors = face(face ~= vertexIdx);      
    neighborIndices = [neighborIndices, neighbors];      
end      
      
neighborIndices = unique(neighborIndices);      
neighborVertices = vertices(neighborIndices, :);      
end 
 
function newPosition = fitPlaneAndProject(originalPos, neighborVertices) 
% Fit best plane to neighbor vertices and project original position onto it 
 
centroid = mean(neighborVertices, 1);      
centeredPoints = neighborVertices - centroid;      
      
[~, ~, V] = svd(centeredPoints, 'econ');      
normal = V(:, end);      
      
pointToCentroid = originalPos - centroid;      
distance = dot(pointToCentroid, normal');      
newPosition = originalPos - distance * normal';      
end 
 
function params = parameterSelectionUI() 
% Create a figure for parameter selection 
fig = figure('Name', 'Parameter Selection', 'NumberTitle', 'off', ... 
'Position', [300 300 500 600], 'MenuBar', 'none', 'ToolBar', 'none'); 
 
% Default values    
params = struct();    
params.gridStep = 2;    
params.max_edge_length = 4;    
params.z_deep = 20;    
params.planeDistance = 50;    
params.weldingAngle = 40;    
params.peakThreshold = 0.005;    
params.enableWelding = false;    
params.enableInterpolation = true;  
params.enableDownsampling = true;  
params.interpolationMethod = 'nearest'; 
params.extrapolationMethod = 'none'; 
params.skipAutoInverse = false;
params.skipManualInverse = false;

% UI Controls    
y_start = 550;    
h = 25;    
spacing = 35;    
    
% Grid Step    
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Grid Step:', 'HorizontalAlignment', 'right');    
edit_gridStep = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], 'String', num2str(params.gridStep));    
    
y_start = y_start - spacing;    
% Max Edge Length    
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Max Edge Length:', 'HorizontalAlignment', 'right');    
edit_edge = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], 'String', num2str(params.max_edge_length));    
    
y_start = y_start - spacing;    
% Z Deep    
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Z Deep:', 'HorizontalAlignment', 'right');    
edit_zdeep = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], 'String', num2str(params.z_deep));    
    
y_start = y_start - spacing;    
% Plane Distance    
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Plane Distance:', 'HorizontalAlignment', 'right');    
edit_planeDist = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], 'String', num2str(params.planeDistance));    
    
y_start = y_start - spacing * 1.5;    
% Enable Welding Checkbox    
chk_welding = uicontrol('Style', 'checkbox', 'Position', [220 y_start 200 h], 'String', 'Enable Mesh Welding', 'Value', params.enableWelding, ...    
    'Callback', @toggleWeldingParams);    
    
y_start = y_start - spacing;    
% Welding Angle (Conditional)    
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Welding Angle:', 'HorizontalAlignment', 'right');    
edit_weldAngle = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], 'String', num2str(params.weldingAngle), 'Enable', 'off');    
    
y_start = y_start - spacing;    
% Peak Threshold (Conditional)    
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Peak Threshold:', 'HorizontalAlignment', 'right');    
edit_peakThresh = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], 'String', num2str(params.peakThreshold), 'Enable', 'off');    
    
y_start = y_start - spacing * 1.5;    
% Enable Downsampling Checkbox    
chk_downsample = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], 'String', 'Enable Downsampling & Denoising', 'Value', params.enableDownsampling);    
    
y_start = y_start - spacing;    
% Enable Interpolation Checkbox    
chk_interp = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], 'String', 'Enable Resampling (Interpolation)', 'Value', params.enableInterpolation, ... 
    'Callback', @toggleInterpParams);    
    
y_start = y_start - spacing; 
% Interpolation Method 
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Interpolation Method:', 'HorizontalAlignment', 'right'); 
popup_interp = uicontrol('Style', 'popupmenu', 'Position', [220 y_start 150 h], ... 
    'String', {'linear', 'nearest', 'natural'}, 'Value', 2); % Default nearest 
 
y_start = y_start - spacing; 
% Extrapolation Method 
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Extrapolation Method:', 'HorizontalAlignment', 'right'); 
popup_extrap = uicontrol('Style', 'popupmenu', 'Position', [220 y_start 150 h], ... 
    'String', {'linear', 'nearest', 'none'}, 'Value', 3); % Default none 
 
y_start = y_start - spacing * 1.5;
% Skip Manual Inverse Rotation Checkbox
chk_skipManualInv = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Skip Manual Inverse Rotation', 'Value', params.skipManualInverse);

y_start = y_start - spacing;
% Skip Automatic Inverse Rotation Checkbox
chk_skipAutoInv = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Skip Automatic Inverse Rotation', 'Value', params.skipAutoInverse);

y_start = y_start - spacing * 2;    
% Start Button    
uicontrol('Style', 'pushbutton', 'Position', [150 y_start 200 40], 'String', 'Start Processing', 'FontSize', 12, 'Callback', @startProcessing);    
    
% Callback to toggle welding parameter inputs    
    function toggleWeldingParams(src, ~)    
        state = get(src, 'Value');    
        if state    
            set(edit_weldAngle, 'Enable', 'on');    
            set(edit_peakThresh, 'Enable', 'on');    
        else    
            set(edit_weldAngle, 'Enable', 'off');    
            set(edit_peakThresh, 'Enable', 'off');    
        end    
    end    
 
% Callback to toggle interpolation parameter inputs 
    function toggleInterpParams(src, ~) 
        state = get(src, 'Value'); 
        if state 
            set(popup_interp, 'Enable', 'on'); 
            set(popup_extrap, 'Enable', 'on'); 
        else 
            set(popup_interp, 'Enable', 'off'); 
            set(popup_extrap, 'Enable', 'off'); 
        end 
    end 
    
% Callback to start processing    
    function startProcessing(~, ~)    
        params.gridStep = str2double(get(edit_gridStep, 'String'));    
        params.max_edge_length = str2double(get(edit_edge, 'String'));    
        params.z_deep = str2double(get(edit_zdeep, 'String'));    
        params.planeDistance = str2double(get(edit_planeDist, 'String'));    
        params.enableWelding = get(chk_welding, 'Value');    
        params.weldingAngle = str2double(get(edit_weldAngle, 'String'));    
        params.peakThreshold = str2double(get(edit_peakThresh, 'String'));    
        params.enableDownsampling = get(chk_downsample, 'Value');  
        params.enableInterpolation = get(chk_interp, 'Value');    
         
        interpOpts = {'linear', 'nearest', 'natural'}; 
        extrapOpts = {'linear', 'nearest', 'none'}; 
        params.interpolationMethod = interpOpts{get(popup_interp, 'Value')}; 
        params.extrapolationMethod = extrapOpts{get(popup_extrap, 'Value')}; 
        params.skipManualInverse = get(chk_skipManualInv, 'Value');
        params.skipAutoInverse = get(chk_skipAutoInv, 'Value');
            
        uiresume(fig);    
        close(fig);    
    end    
    
% Wait for user input    
uiwait(fig);    
end 
