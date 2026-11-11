clear 
clc 
close all 
 
%% FILE SELECTION UI 
% Create a UI to select the input file and its type 
[filePath, fileType, useRGB] = fileSelectionUI(); 
 
% Check if a file was selected 
if isempty(filePath) 
    fprintf('No file selected. Exiting script.\n'); 
    return; 
end 
 
% Split into folder, name, extension 
[folder, name, ~] = fileparts(filePath); 
% Build new filename: original_name + "_srf" + .stl 
stl_out = fullfile(folder, name + ".stl"); 
pc_out = fullfile(folder, name + "ptcloud.ply"); 
 
%% LOAD DATA BASED ON SELECTION 
fprintf('Loading data...\n');
if strcmp(fileType, 'PointCloud') 
    fprintf('Loading Point Cloud file: %s\n', filePath); 
    try
        T = readmatrix(filePath); 
    catch ME
        error('Failed to read point cloud file: %s', ME.message);
    end
    
    % Validate data
    if isempty(T) || size(T, 1) < 3
        error('Point cloud file is empty or has insufficient points (need at least 3 points)');
    end
    
    if size(T, 2) < 3
        error('Point cloud file must have at least 3 columns (X, Y, Z)');
    end
 
    % Check for RGB data   
    colorData = [];   
    if useRGB && size(T, 2) >= 6   
        % Assume columns 4, 5, 6 are R, G, B   
        RGB = T(:, 4:6);   
        % Normalize if needed (assuming 0-255 if max > 1)   
        if max(RGB(:)) > 1   
            RGB = double(RGB) / 255;   
        end   
        colorData = RGB;   
    end   
       
    % Create the point cloud      
    ptCloud = pointCloud([T(:,1) T(:,2) T(:,3)]);      
    if ~isempty(colorData)   
        ptCloud.Color = uint8(colorData * 255);   
    end   
       
    % Create a logical index for values not equal to -9999      
    validIndices = T(:,3) >= -9999;      
       
    % Filter the point cloud      
    filteredPtCloud = pointCloud(ptCloud.Location(validIndices, :));   
    if ~isempty(colorData)   
        colorData = colorData(validIndices, :);   
    end   
    
    % Validate after filtering
    if filteredPtCloud.Count < 3
        error('After filtering invalid points, insufficient data remains (< 3 points)');
    end
    fprintf('Point cloud loaded: %d points\n', filteredPtCloud.Count);
    
elseif strcmp(fileType, 'Raster') 
    fprintf('Loading Raster file: %s\n', filePath); 
    % Use the provided code for raster loading 
    georaster = filePath; 
    try
        [A,R] = readgeoraster(georaster,"OutputType","double"); 
    catch ME
        error('Failed to read raster file: %s', ME.message);
    end
    
    % Validate raster data
    if isempty(A) || all(A(:) <= 0)
        error('Raster file is empty or contains no valid elevation data');
    end
 
    %% GEOGRAPHIC COORDINATES     
    x_step = (R.XWorldLimits(2)-R.XWorldLimits(1))./(R.RasterSize(2)-1);     
    y_step = (R.YWorldLimits(2)-R.YWorldLimits(1))./(R.RasterSize(1)-1);     
    x_R = R.XWorldLimits(1)+1./2*x_step:x_step:R.XWorldLimits(2)+1./2*x_step;     
    y_R = R.YWorldLimits(1)+1./2*y_step:y_step:R.YWorldLimits(2)+1./2*y_step;     
         
    %%     
    Y_R = zeros(1,R.RasterSize(1).*R.RasterSize(2));     
    X_R = zeros(1,R.RasterSize(1).*R.RasterSize(2));     
    HEIGHT = zeros(1,R.RasterSize(1).*R.RasterSize(2));     
    h = zeros(R.RasterSize(1),R.RasterSize(2));     
         
    n = 1;     
    for i = 1:R.RasterSize(1)     
        for j = 1:R.RasterSize(2)     
            Y_R(n)=y_R(i);     
            X_R(n)=x_R(j);     
            if A(i,j)<=0     
                h(i,j)=-1;     
                HEIGHT(n) = -1;     
            else     
                h(i,j)=A(i,j);     
                HEIGHT(n) = A(i,j);     
            end     
            n=n+1;     
        end     
    end     
         
    %%     
    % Create the point cloud     
    ptCloud = pointCloud([X_R',flipud(Y_R'), HEIGHT']);     
         
    %%     
    % Create a logical index for values not equal to -9999 (or >=0 as per raster code)   
    validIndices = HEIGHT >= 0;     
         
    % Filter the point cloud     
    filteredPtCloud = pointCloud(ptCloud.Location(validIndices, :));   
       
    % Validate after filtering
    if filteredPtCloud.Count < 3
        error('After filtering invalid elevation values, insufficient data remains (< 3 points)');
    end
    fprintf('Raster loaded: %d valid points\n', filteredPtCloud.Count);
    
    colorData = [];   
    if useRGB   
        fprintf('Generating hillshade...\n');
        % Calculate Hillshade   
        [dx, dy] = gradient(h, x_step, y_step);   
        slope = atan(sqrt(dx.^2 + dy.^2));   
        aspect = atan2(-dy, -dx); % Aspect direction    
           
        % Parameters   
        azimuth = 315;   
        altitude = 45;   
           
        az_rad = deg2rad(azimuth);   
        alt_rad = deg2rad(altitude);   
           
        % Hillshade calculation   
        hillshade = 255 * ( (sin(alt_rad) .* cos(slope)) + (cos(alt_rad) .* sin(slope) .* cos(az_rad - aspect)) );   
        hillshade(hillshade < 0) = 0;   
           
        % Flatten   
        HS_flat = zeros(1, R.RasterSize(1).*R.RasterSize(2));   
        n = 1;   
        for i = 1:R.RasterSize(1)   
            for j = 1:R.RasterSize(2)   
                HS_flat(n) = hillshade(i,j);   
                n = n + 1;   
            end   
        end   
           
        % Filter using validIndices   
        colorData = HS_flat(validIndices)';   
    end   
else 
    error('Unknown file type selected.'); 
end 
 
%% METHOD AND PARAMETER SELECTION
fprintf('\n=== METHOD AND PARAMETER SELECTION ===\n');

% Step 1: choose meshing method
selectedMethod = methodSelectionUI();
if isempty(selectedMethod)
    fprintf('No method selected. Exiting.\n');
    return;
end
fprintf('Selected method: %s\n', selectedMethod);

% Step 2: method-specific parameter panel
if strcmp(selectedMethod, 'Poisson')
    params = poissonParameterUI();
else
    params = delaunayParameterUI();
end
params.usePoissonRecon = strcmp(selectedMethod, 'Poisson');

gridStep        = params.gridStep;
max_edge_length = params.max_edge_length;
z_deep          = params.z_deep;
interpolation   = params.interpolationMethod;
extrapolation   = params.extrapolationMethod;
refcycle        = params.refcycle;

% Distance between cutting planes (in meters)
planeDistance = params.planeDistance;

%% INTERACTIVE PLANE SELECTION (MOVED HERE - RIGHT AFTER READING POINT CLOUD)
% Get initial point cloud data (kept intact so the plane can be re-selected)
fullPtCloud = filteredPtCloud;
x_initial = fullPtCloud.Location(:,1);
y_initial = fullPtCloud.Location(:,2);
z_initial = fullPtCloud.Location(:,3);

% Plane selection -> cut -> alignment -> manual rotation. The manual
% rotation window can send the user back here to choose a new plane.
planeState = [];
% Last confirmed planes, shared between runs ("Load Last Planes" button)
lastPlaneFile = fullfile(fileparts(mfilename('fullpath')), 'last_plane_state.mat');
while true
    fprintf('\n=== PLANE SELECTION ===\n');

    % Call interactive plane selector with editable distance (restores the previous choice when going back)
    [planePosition, planeNormal, planeDistance, targetAxis, planeState] = interactivePlaneSelector(x_initial, y_initial, z_initial, planeDistance, colorData, useRGB, planeState, lastPlaneFile);
    try
        save(lastPlaneFile, 'planeState');
    catch ME
        warning('Could not save last plane state to %s: %s', lastPlaneFile, ME.message);
    end
     
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
     
    % Override cutting if requested (Skip Plane Cutting, but keep alignment) 
    if isfield(params, 'skipPlaneCutting') && params.skipPlaneCutting 
        fprintf('Skipping Plane Cutting as requested (Keeping Alignment). All points retained.\n'); 
        cutIndices = true(length(x_initial), 1); 
    end 
     
    fprintf('Original points: %d\n', length(x_initial)); 
    fprintf('Points after plane cutting: %d\n', sum(cutIndices)); 
     
    % Validate cut results
    if sum(cutIndices) < 3
        warning('Plane cutting left fewer than 3 points. Adjust plane position or distance.');
        continue;
    end

    % Apply the cut to the point cloud
    filteredPtCloud = pointCloud(fullPtCloud.Location(cutIndices, :));
     
    %% ROTATION AROUND Z-AXIS ONLY - TO ALIGN WITH X OR Y AXIS 
    fprintf('\n=== ALIGNMENT ROTATION ===\n');
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
        rotation_angle_deg = 0; 
        fprintf('Target axis is Minimal or unset - no alignment rotation applied\n'); 
    else 
        % Calculate Strike Azimuth from Dip Direction 
        strikeAz = dipDir; 
         
        % Rotation required (CCW from +X). Azimuths grow clockwise, so the CCW
        % angle that brings strikeAz onto targetAz is strikeAz - targetAz
        rotation_angle_deg = strikeAz - targetAz;
             
        % Convert to radians      
        angle_z = deg2rad(rotation_angle_deg);      
          
        % Rotation matrix around Z-axis      
        R = [cos(angle_z), -sin(angle_z), 0;      
             sin(angle_z),  cos(angle_z), 0;      
             0,             0,             1];      
               
        fprintf('Aligning Strike (%.2f°) to Target (%.2f°)\n', strikeAz, targetAz);      
        fprintf('Rotation around Z-axis: %.2f degrees\n', rotation_angle_deg);      
    end 
     
    % Create the affine transformation 
    affine_matrix = eye(4); 
    affine_matrix(1:3, 1:3) = R; % affinetform3d uses column vectors: A = [R t; 0 0 0 1]
    tform_align = affinetform3d(affine_matrix); 
     
    % Apply the transformation 
    ptCloud_rotated = pctransform(filteredPtCloud, tform_align); 
     
    % --- Translation to Origin (AFTER Rotation) --- 
    translation_vec = [0, 0, 0]; 
    if isfield(params, 'translateToOrigin') && params.translateToOrigin 
        % Check if point cloud has valid data 
        if ~isempty(ptCloud_rotated.Location) 
            % Calculate translation vector (centroid to origin), ignoring NaNs 
            translation_vec = -mean(ptCloud_rotated.Location, 1, 'omitnan'); 
         
            if any(isnan(translation_vec))     
                translation_vec = [0, 0, 0];     
                warning('Calculated translation vector contains NaNs. Skipping translation.');     
            else     
                fprintf('Translating point cloud to origin. Vector: [%.4f, %.4f, %.4f]\n', translation_vec);     
            end     
        else     
            translation_vec = [0, 0, 0];     
            warning('Point cloud is empty. Skipping translation.');     
        end     
             
        % Create translation transform     
        trans_matrix = eye(4);     
        trans_matrix(1:3, 4) = translation_vec;     
        tform_trans = affinetform3d(trans_matrix);     
         
        % Apply translation     
        if ~all(translation_vec == 0)     
            ptCloud_rotated = pctransform(ptCloud_rotated, tform_trans);     
        end     
             
        % Alignment first, then translation (column-vector convention: A_trans * A_align)
        tform_combined = affinetform3d(tform_trans.A * tform_align.A);
             
        % tform_back is the inverse of the total automatic alignment/translation     
        tform_back = invert(tform_combined);     
    else 
        % Store the inverse transformation for rotating back later 
        tform_back = invert(tform_align); 
    end 
     
    % Save to file 
    fprintf('Saving plane coordinates...\n');
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
         
        fprintf(fid, '\n--- Alignment Transformation Info ---\n');     
        fprintf(fid, 'Rotation Matrix (R):\n');     
        fprintf(fid, '%.6f %.6f %.6f\n', R(1,:));     
        fprintf(fid, '%.6f %.6f %.6f\n', R(2,:));     
        fprintf(fid, '%.6f %.6f %.6f\n', R(3,:));     
             
        fprintf(fid, 'Translation Vector (Post-Rotation): %.6f, %.6f, %.6f\n', translation_vec);     
             
        fclose(fid);      
        fprintf('Plane coordinates saved to %s\n', paramFile);      
    else 
        fprintf('Error: Could not open %s for writing.\n', paramFile); 
    end 
     
    %% OPTIONAL MANUAL ROTATION 
    fprintf('\n=== MANUAL ROTATION ===\n');
    % Allow user to perform an additional rotation interactively 
    [tform_manual, goBackToPlane] = interactiveRotationSelector(ptCloud_rotated);
    if goBackToPlane
        fprintf('Returning to plane selection...\n');
        continue;
    end
    ptCloud_rotated = pctransform(ptCloud_rotated, tform_manual);
    break;
end
 
 
figure(2)
pcshow(ptCloud_rotated);
title('Automatically Rotated DTM');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
drawnow;

% Compute PoissonRecon availability once here — used to branch all downstream steps
poissonExePath = params.poissonReconPath;
poissonAvailable = ~isempty(strtrim(poissonExePath)) && exist(poissonExePath, 'file');

%% PRE-PROCESSING — two separate paths
fprintf('\n=== PRE-PROCESSING ===\n');

if params.usePoissonRecon && poissonAvailable
    %% --- POISSON PATH: statistical outlier removal only ---
    fprintf('PoissonRecon path: statistical outlier removal only.\n');
    ptCloudWithGround_Out = removeInvalidPoints(pcdenoise(ptCloud_rotated, ...
        'NumNeighbors', params.outlierNumNeighbors, 'Threshold', params.outlierThreshold));
    if ptCloudWithGround_Out.Count < 3
        error('After outlier removal, insufficient points remain (< 3)');
    end
    fprintf('Points after outlier removal: %d\n', ptCloudWithGround_Out.Count);
    figure(3)
    pcshow(ptCloudWithGround_Out.Location)
    title('Point Cloud — Outlier Removal (PoissonRecon path)')
    drawnow;

else
    %% --- DELAUNAY PATH: downsampling, denoising, then overlap/dedup filters ---
    if isfield(params, 'enableDownsampling') && params.enableDownsampling
        fprintf('Downsampling with grid step: %.2f\n', gridStep);
        ptCloud_down = pcdownsample(ptCloud_rotated, 'gridAverage', gridStep);
        fprintf('Denoising...\n');
        ptCloud_denoised = pcdenoise(ptCloud_down, ...
            'NumNeighbors', params.outlierNumNeighbors, 'Threshold', params.outlierThreshold);
        ptCloudWithGround_Out = removeInvalidPoints(ptCloud_denoised);
        if ptCloudWithGround_Out.Count < 3
            error('After downsampling and denoising, insufficient points remain (< 3)');
        end
        fprintf('Points after downsampling: %d\n', ptCloudWithGround_Out.Count);
        figure(3)
        pcshow(ptCloudWithGround_Out.Location)
        title('Downsampled Rotated Ground Points (Delaunay path)')
        drawnow;
    else
        ptCloudWithGround_Out = removeInvalidPoints(ptCloud_rotated);
        if ptCloudWithGround_Out.Count < 3
            error('After removing invalid points, insufficient points remain (< 3)');
        end
        figure(3)
        pcshow(ptCloudWithGround_Out.Location)
        title('Rotated Ground Points — Original Resolution (Delaunay path)')
        drawnow;
    end
end 
 
%% PREPARE FOR MESH CREATION
fprintf('\n=== PREPARING MESH ===\n');
% Procedure for create .stl file 
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

% Validate after finite filtering
if length(x_filtered) < 3
    error('After filtering non-finite values, insufficient points remain (< 3)');
end
fprintf('Valid finite points: %d\n', length(x_filtered));

% Filter Overlapping Points in Z-direction — Delaunay only.
% Skipped for PoissonRecon because it removes points at the same XY but
% different Z, destroying the vertical/overhanging geometry that PoissonRecon needs.
if isfield(params, 'enableOverlapFilter') && params.enableOverlapFilter && ~(params.usePoissonRecon && poissonAvailable)
    overlapRadius = params.overlapRadius;
    if overlapRadius > 0
        fprintf('Applying overlap filter (Radius: %.4f)...\n', overlapRadius);
        
        % Data is currently in x_filtered, y_filtered, z_filtered arrays
        % Combine into matrix
        pts_xyz = [x_filtered, y_filtered, z_filtered];
        numPts = size(pts_xyz, 1);
        
        % Sort by Z ascending (lowest point first — will be kept)
        [~, sortIdx] = sort(pts_xyz(:,3), 'ascend');
        sortedPts = pts_xyz(sortIdx, :);

        % We will use a boolean mask to mark points to keep
        % Build KDTree on 2D coordinates (X, Y)
        Mdl = KDTreeSearcher(sortedPts(:, 1:2));

        toRemove = false(size(sortedPts, 1), 1);

        % Progress tracking
        progressStep = max(1, floor(numPts / 20)); % Update every 5%
        fprintf('Progress: ');

        % Efficiently find neighbors for all points
        for i = 1:size(sortedPts, 1)
            if toRemove(i)
                continue;
            end

            % Progress indicator
            if mod(i, progressStep) == 0
                fprintf('.');
            end

            % Find neighbors within radius
            idxs = rangesearch(Mdl, sortedPts(i, 1:2), overlapRadius);
            neighborIdxs = idxs{1};

            % Mark neighbors that are higher (index > i) as to remove
            neighborsToRemove = neighborIdxs(neighborIdxs > i);
            toRemove(neighborsToRemove) = true;
        end
        fprintf(' Done!\n');
        
        % Filter the points
        keptSortedIndices = find(~toRemove);
        keptOriginalIndices = sortIdx(keptSortedIndices);
        
        x_filtered = x_filtered(keptOriginalIndices);
        y_filtered = y_filtered(keptOriginalIndices);
        z_filtered = z_filtered(keptOriginalIndices);
        
        fprintf('Points after overlap filtering: %d\n', length(x_filtered));
        
        % Validate after overlap filtering
        if length(x_filtered) < 3
            error('After overlap filtering, insufficient points remain (< 3)');
        end
    else
        fprintf('Overlap radius is 0 or negative. Skipping filter.\n');
    end
end
 
% Remove duplicates in XY before triangulation — Delaunay only.
% Skipped for PoissonRecon: collapsing points to mean Z destroys vertical
% and overhanging geometry that PoissonRecon needs.
if ~(params.usePoissonRecon && poissonAvailable)
    fprintf('Removing XY duplicates...\n');
    [xy_unique, ~, ic] = unique([x_filtered(:), y_filtered(:)], 'rows', 'stable');
    z_unique = accumarray(ic, z_filtered, [], @mean);
    x_filtered = xy_unique(:, 1);
    y_filtered = xy_unique(:, 2);
    z_filtered = z_unique;
    fprintf('Points after deduplication: %d\n', length(x_filtered));
    if length(x_filtered) < 3
        error('After deduplication, insufficient points remain (< 3)');
    end
end
 
%% SAVE POINT CLOUD
fprintf('Saving point cloud to file...\n');
ptCloudOuttxt = pointCloud([x_filtered y_filtered z_filtered]);
pcwrite(ptCloudOuttxt, pc_out);
fprintf('Point cloud saved to %s\n', pc_out);

%% -----------------------------------------------------------------------
%  MESH GENERATION — PoissonRecon first, Delaunay as fallback
%% -----------------------------------------------------------------------
fprintf('\n=== MESH GENERATION ===\n');

vertices_top_o = [x_filtered, y_filtered, z_filtered];
vertices_top   = vertices_top_o;
poissonSuccess = false;

% --- Try PoissonRecon ---
% (poissonExePath and poissonAvailable already computed before overlap filter)
if params.usePoissonRecon && poissonAvailable
    fprintf('PoissonRecon executable found. Running Poisson Surface Reconstruction...\n');
    [vertices_top, faces_t, poissonSuccess] = runPoissonRecon( ...
        [x_filtered, y_filtered, z_filtered], folder, name, ...
        poissonExePath, params.poissonDepth, params.poissonPointWeight, ...
        params.poissonNormalNeighbors, params.enablePoissonTrimmer, params.poissonTrimValue);
    if poissonSuccess
        fprintf('PoissonRecon complete: %d vertices, %d faces\n', ...
            size(vertices_top,1), size(faces_t,1));

        % Remove isolated floating components — keep only the largest connected part
        fprintf('  Removing isolated components...\n');
        [vertices_top, faces_t] = removeIsolatedComponents(vertices_top, faces_t);

        faces_t_o = faces_t;
        fprintf('  Final PoissonRecon mesh: %d vertices, %d faces\n', ...
            size(vertices_top,1), size(faces_t,1));
    else
        warning('MESH:poissonFallback', ...
            'PoissonRecon was selected but failed during reconstruction.\nFalling back to Delaunay triangulation.\nNote: vertical faces and overhangs will NOT be reconstructed correctly.');
    end
elseif params.usePoissonRecon && ~poissonAvailable
    warning('MESH:poissonFallback', ...
        'PoissonRecon was selected but the executable was not found at:\n  %s\nFalling back to Delaunay triangulation.\nNote: vertical faces and overhangs will NOT be reconstructed correctly.', ...
        poissonExePath);
end

% --- Delaunay fallback ---
if ~poissonSuccess
    fprintf('\n--- Delaunay Triangulation ---\n');

    %% INITIAL TRIANGULATION
    fprintf('Creating initial Delaunay triangulation...\n');
    try
        dt_filtered = delaunayTriangulation(x_filtered, y_filtered);
    catch ME
        error('Failed to create Delaunay triangulation: %s', ME.message);
    end
    faces = dt_filtered.ConnectivityList;
    if isempty(faces)
        error('Delaunay triangulation produced no faces');
    end
    fprintf('Initial triangulation: %d faces\n', size(faces, 1));

    vertices_top_o = [x_filtered, y_filtered, z_filtered];
    vertices_top   = vertices_top_o;

    fprintf('Filtering triangles by edge length (max: %.2f)...\n', max_edge_length);
    faces_t_o = filter_by_length(faces, vertices_top_o(:,1), vertices_top_o(:,2), max_edge_length);
    if isempty(faces_t_o)
        error('All triangles removed by edge length filter. Consider increasing max_edge_length.');
    end
    fprintf('Faces after edge length filtering: %d\n', size(faces_t_o, 1));

    %% MESH WELDING
    fprintf('\n=== MESH WELDING ===\n');
    if params.enableWelding
        weldingAngle  = params.weldingAngle;
        peakThreshold = params.peakThreshold;
        coplanarAngle = 60;
        maxIterations = 10;
        ncycle = 3;
        n = 1;
        faces = faces_t_o;
        while n <= ncycle
            fprintf('Welding cycle %d/%d...\n', n, ncycle);
            [vertices_top, faces, modifiedIndices, stats] = advancedMeshWelding(...
                vertices_top, faces, weldingAngle, coplanarAngle, maxIterations, peakThreshold);
            if isempty(faces) || isempty(vertices_top)
                warning('Mesh welding removed all geometry at cycle %d. Reverting.', n);
                vertices_top = vertices_top_o;
                faces = faces_t_o;
                break;
            end
            n = n + 1;
        end
        fprintf('Mesh welding complete. Vertices: %d, faces: %d\n', size(vertices_top,1), size(faces,1));
    else
        faces = faces_t_o;
        fprintf('Mesh welding disabled.\n');
    end

    %% INTERPOLATION/RESAMPLING
    fprintf('\n=== INTERPOLATION ===\n');
    x_filtered = vertices_top(:,1);
    y_filtered = vertices_top(:,2);
    z_filtered = vertices_top(:,3);

    if params.enableInterpolation
        i = 1;
        while i <= refcycle
            fprintf('Interpolation cycle %d/%d...\n', i, refcycle);
            if length(x_filtered) < 3
                warning('Insufficient points for interpolation at cycle %d. Skipping.', i);
                break;
            end
            try
                F = scatteredInterpolant(x_filtered, y_filtered, z_filtered, interpolation, extrapolation);
            catch ME
                warning('Failed to create interpolant at cycle %d: %s. Skipping.', i, ME.message);
                break;
            end
            x_array = min(x_filtered):gridStep/2:max(x_filtered);
            y_array = min(y_filtered):gridStep/2:max(y_filtered);
            [Xq, Yq] = meshgrid(x_array, y_array);
            Zq = F(Xq, Yq);
            x_array_fine = min(x_filtered):gridStep/2:max(x_filtered);
            y_array_fine = min(y_filtered):gridStep/2:max(y_filtered);
            [Xq_c, Yq_c] = meshgrid(x_array_fine, y_array_fine);
            valid = ~isnan(Zq(:));
            if sum(valid) < 10
                warning('Insufficient valid data for interpolation at cycle %d. Skipping.', i);
                break;
            end
            try
                F_cubic = scatteredInterpolant(Xq(valid), Yq(valid), Zq(valid), interpolation, extrapolation);
                Zq_c = F_cubic(Xq_c, Yq_c);
            catch ME
                warning('Failed to create refined interpolant at cycle %d: %s. Skipping.', i, ME.message);
                break;
            end
            x_filtered = Xq_c(:);
            y_filtered = Yq_c(:);
            z_filtered = Zq_c(:);
            validIdx = isfinite(z_filtered) & isfinite(x_filtered) & isfinite(y_filtered);
            fprintf('  Total: %d  Valid: %d  Removed: %d\n', ...
                length(z_filtered), sum(validIdx), sum(~validIdx));
            if sum(validIdx) < 3
                warning('Insufficient valid points after filtering at cycle %d. Skipping.', i);
                break;
            end
            x_filtered = x_filtered(validIdx);
            y_filtered = y_filtered(validIdx);
            z_filtered = z_filtered(validIdx);
            i = i + 1;
        end
        fprintf('Interpolation complete.\n');
    else
        fprintf('Interpolation disabled.\n');
    end

    %% SECOND TRIANGULATION
    fprintf('\n=== FINAL TRIANGULATION ===\n');
    fprintf('Removing duplicates before final triangulation...\n');
    [xy_unique2, ~, ic2] = unique([x_filtered(:), y_filtered(:)], 'rows', 'stable');
    z_unique2 = accumarray(ic2, z_filtered, [], @mean);
    x_filtered = xy_unique2(:, 1);
    y_filtered = xy_unique2(:, 2);
    z_filtered = z_unique2;
    fprintf('Points after second deduplication: %d\n', length(x_filtered));
    if length(x_filtered) < 3
        error('After second deduplication, insufficient points remain for triangulation (< 3)');
    end
    try
        DT = delaunayTriangulation(x_filtered, y_filtered);
    catch ME
        error('Failed to create final Delaunay triangulation: %s', ME.message);
    end
    if isempty(DT.ConnectivityList)
        error('Final Delaunay triangulation produced no faces');
    end
    vertices_top = [x_filtered, y_filtered, z_filtered];
    faces_obj = triangulation(DT.ConnectivityList, vertices_top);
    fprintf('Filtering final triangulation by edge length...\n');
    faces_t = filter_by_length(faces_obj.ConnectivityList, vertices_top(:,1), vertices_top(:,2), max_edge_length);
    if isempty(faces_t)
        error('All final triangles were removed by edge length filter');
    end
    fprintf('Final mesh: %d vertices, %d faces\n', size(vertices_top,1), size(faces_t,1));
    faces_t_o = faces_t;
end  % end Delaunay fallback

%% CREATE 3D OBJECT
fprintf('\n=== CREATING 3D MESH ===\n');
z_flat = min(vertices_top(:,3)) - z_deep;

if poissonSuccess
    % PoissonRecon already produces a watertight closed surface.
    fprintf('Using PoissonRecon watertight mesh directly.\n');
    TR = triangulation(faces_t, vertices_top);
elseif isfield(params, 'createSolidMesh') && ~params.createSolidMesh
    % Delaunay - surface only
    fprintf('Generating Surface Mesh Only (Skipping Extrusion)...\n');
    TR = triangulation(faces_t, vertices_top);
else
    % Delaunay - solid block
    fprintf('Generating Solid Block Mesh...\n');
    vertices_bottom = [vertices_top(:,1), vertices_top(:,2), ...
                       z_flat * ones(size(vertices_top,1), 1)];
    try
        dt = delaunayTriangulation(vertices_top(:,1), vertices_top(:,2));
    catch ME
        error('Failed to create bottom triangulation: %s', ME.message);
    end
    T_filtered = dt.ConnectivityList;
    fprintf('Filtering bottom triangulation by edge length...\n');
    T_filtered = filter_by_length(T_filtered, vertices_top(:,1), vertices_top(:,2), max_edge_length);
    if isempty(T_filtered)
        error('All bottom triangles were removed by edge length filter');
    end
    vertices_all = [vertices_top; vertices_bottom];
    n_top = size(vertices_top, 1);
    faces_bottom = fliplr(T_filtered + n_top);
    dt_filtered_faces = triangulation(T_filtered, vertices_top(:,1), vertices_top(:,2));
    boundary_edges = freeBoundary(dt_filtered_faces);
    if isempty(boundary_edges)
        warning('No boundary edges found. Mesh may not be properly closed.');
    end
    num_edges = size(boundary_edges, 1);
    faces_side = zeros(2 * num_edges, 3);
    for i = 1:num_edges
        v1  = boundary_edges(i, 1);
        v2  = boundary_edges(i, 2);
        v1b = v1 + n_top;
        v2b = v2 + n_top;
        faces_side(2*i - 1, :) = [v1, v1b, v2b];
        faces_side(2*i,     :) = [v1, v2b, v2];
    end
    faces_all = [faces_t; faces_bottom; faces_side];
    fprintf('Total mesh: %d vertices, %d faces (top: %d, bottom: %d, sides: %d)\n', ...
        size(vertices_all,1), size(faces_all,1), ...
        size(faces_t,1), size(faces_bottom,1), size(faces_side,1));
    TR = triangulation(faces_all, vertices_all);
end

%% ROTATE MESH BACK TO ORIGINAL POSITION
fprintf('\n=== ROTATING MESH BACK ===\n');
TR_rotated_points = TR.Points;

% 1. Inverse Manual Rotation
if isfield(params, 'skipManualInverse') && params.skipManualInverse
    fprintf('Skipping Manual Inverse Rotation.\n');
else
    fprintf('Applying inverse manual rotation...\n');
    TR_rotated_points = transformPointsForward(invert(tform_manual), TR_rotated_points);
end

% 2. Inverse Automatic Alignment
if isfield(params, 'skipAutoInverse') && params.skipAutoInverse
    fprintf('Skipping Automatic Inverse Rotation (Alignment).\n');
else
    fprintf('Applying inverse automatic rotation...\n');
    TR_rotated_points = transformPointsForward(tform_back, TR_rotated_points); % undoes alignment and translation
end

TR_rotated = triangulation(TR.ConnectivityList, TR_rotated_points);

%% EXPORT AND VISUALIZE
fprintf('\n=== EXPORTING RESULTS ===\n');
fprintf('Writing STL file to: %s\n', stl_out);
try
    stlwrite(TR_rotated, stl_out);
    fprintf('STL file successfully written!\n');
catch ME
    error('Failed to write STL file: %s', ME.message);
end

% Visualize
fprintf('Generating visualization...\n');
figure(4)
subplot(1,2,1);
if poissonSuccess
    % PoissonRecon generates its own vertex set — use vertices_top, not vertices_top_o
    trisurf(faces_t_o, vertices_top(:,1), vertices_top(:,2), vertices_top(:,3), 'FaceAlpha', 0.8);
    title('PoissonRecon Mesh (aligned space)');
else
    trisurf(faces_t_o, vertices_top_o(:,1), vertices_top_o(:,2), vertices_top_o(:,3), 'FaceAlpha', 0.8);
    title('Initial Delaunay Mesh');
end
axis equal; grid on;

subplot(1,2,2);
trisurf(faces_t, vertices_top(:,1), vertices_top(:,2), vertices_top(:,3), 'FaceAlpha', 0.8);
if poissonSuccess
    title('PoissonRecon Mesh (final)');
else
    title('After Processing');
end
axis equal; grid on;

figure(5)
trisurf(TR_rotated, 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');
axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Final Mesh (original coordinate space)');
view(3);

fprintf('\n=== PROCESSING COMPLETE ===\n');
fprintf('Mesher used: %s\n', choose_str(poissonSuccess, 'PoissonRecon', 'Delaunay'));
fprintf('Output files:\n');
fprintf('  - STL: %s\n', stl_out);
fprintf('  - Point Cloud: %s\n', pc_out);
fprintf('  - Plane Info: %s\n', paramFile);
 
%% ========================================================================
%  SUPPORTING FUNCTIONS
%% ========================================================================

%% FILE SELECTION UI FUNCTION 
function [filePath, fileType, useRGB] = fileSelectionUI() 
% Create figure for file selection 
fig = figure('Name', 'Input File Selection', 'NumberTitle', 'off', ... 
    'Position', [300 300 500 250], 'MenuBar', 'none', 'ToolBar', 'none', ... 
    'Resize', 'off'); 
 
% Initialize outputs   
filePath = '';   
fileType = '';   
useRGB = false;   
   
% Default selection   
currentType = 'PointCloud';    
validSelection = false;   
   
% UI Layout   
uicontrol('Style', 'text', 'String', '1. Select Input File Type and File', ...   
    'Position', [50 200 400 30], 'FontSize', 12, 'FontWeight', 'bold');   
   
% Radio Button Group for File Type   
bg = uibuttongroup('Parent', fig, 'Position', [0.1 0.5 0.8 0.25], ...   
    'Title', 'File Type', 'SelectionChangedFcn', @typeChanged);   
   
uicontrol(bg, 'Style', 'radiobutton', 'String', 'Point Cloud (.txt, .csv, .xyz)', ...   
    'Position', [10 10 200 30], 'Tag', 'PointCloud');   
       
uicontrol(bg, 'Style', 'radiobutton', 'String', 'Raster (.tif, .tiff)', ...   
    'Position', [220 10 150 30], 'Tag', 'Raster');   
   
% File Path Display   
uicontrol('Style', 'text', 'String', 'Selected File:', ...   
    'Position', [50 100 80 20], 'HorizontalAlignment', 'left');   
   
edit_path = uicontrol('Style', 'edit', 'Position', [130 100 250 25], ...   
    'String', 'No file selected', 'Enable', 'inactive');   
   
% Browse Button   
uicontrol('Style', 'pushbutton', 'String', '...', ...   
    'Position', [390 100 60 25], 'Callback', @browseFile);   
   
% Checkbox for RGB/Hillshade   
chk_rgb = uicontrol('Style', 'checkbox', 'String', 'Visualize RGB / Hillshade', ...   
    'Position', [50 80 200 20], 'Value', false);   
   
% Start/Confirm Button   
btn_ok = uicontrol('Style', 'pushbutton', 'String', 'Start Processing', ...   
    'Position', [150 30 200 40], 'Enable', 'off', ...   
    'Callback', @confirm);   
   
% Wait for user interaction   
addVerticalScroll(fig);
uiwait(fig);   
   
% Check if we have a valid selection after wait   
if ~validSelection || ~ishandle(fig)   
    % User closed window or cancelled   
    filePath = '';   
    fileType = '';   
end   
   
% Clean up   
if ishandle(fig)   
    delete(fig);   
end   
   
% --- Nested Callbacks ---   
   
    function typeChanged(~, event)   
        currentType = event.NewValue.Tag;   
        % Reset file selection when type changes to avoid mismatch   
        set(edit_path, 'String', 'No file selected');   
        filePath = '';   
        set(btn_ok, 'Enable', 'off');   
    end   
 
    function browseFile(~, ~)   
        if strcmp(currentType, 'PointCloud')   
            filters = {'*.txt;*.csv;*.xyz', 'Point Cloud Files (*.txt, *.csv, *.xyz)'; '*.*', 'All Files'};   
            titleStr = 'Select Point Cloud File';   
        else   
            filters = {'*.tif;*.tiff;*.geotiff', 'Raster Files (*.tif, *.tiff, .geotiff)'; '*.*', 'All Files'};   
            titleStr = 'Select Raster File';   
        end   
           
        [f, p] = uigetfile(filters, titleStr);   
        if f ~= 0   
            selectedPath = fullfile(p, f);   
            set(edit_path, 'String', selectedPath);   
            set(btn_ok, 'Enable', 'on');   
            % Update local variable (will be returned if confirmed)   
            filePath = selectedPath;   
        end   
    end   
 
    function confirm(~, ~)   
        fileType = currentType;   
        validSelection = true;   
        useRGB = get(chk_rgb, 'Value');   
        uiresume(fig);   
    end   
end 
 
%% INTERACTIVE PLANE SELECTOR FUNCTION 
function [planePosition, planeNormal, finalDistance, targetAxis, planeState] = interactivePlaneSelector(x, y, z, initialDistance, colorData, useRGB, initialState, lastPlaneFile)
% initialState (optional): planeState returned by a previous call, used to
% reopen the selector with the same position, rotation, distance and axis.
% lastPlaneFile (optional): .mat file with the planeState confirmed in the
% previous run, loaded by the "Load Last Planes" button.
if nargin < 7, initialState = []; end
if nargin < 8, lastPlaneFile = ''; end
% Create figure for interactive plane selection 
fig = figure('Name', 'Interactive Plane Selector', 'NumberTitle', 'off', ... 
    'Position', [100 100 1400 800]); 
 
% Create main axes for 3D view 
ax = axes('Parent', fig, 'Position', [0.05 0.15 0.7 0.8]); 
 
% Downsample points for visualization 
x_plot = x; 
y_plot = y; 
z_plot = z; 
if ~isempty(colorData) 
    c_plot = colorData; 
else 
    c_plot = z_plot; 
end 
 
% Plot point cloud 
scatter3(ax, x_plot, y_plot, z_plot, 0.5, c_plot, 'filled'); 
if useRGB && ~isempty(colorData) && size(colorData, 2) == 1 
    colormap(ax, 'gray'); 
elseif useRGB && ~isempty(colorData) && size(colorData, 2) == 3 
    % RGB, no colormap needed 
else 
    colormap(ax, 'parula'); 
end 
hold(ax, 'on'); 
axis(ax, 'equal'); 
grid(ax, 'on'); 
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z'); 
title(ax, 'Click and drag arrows to move plane, use sliders to adjust'); 
view(ax, 3); 
rotate3d(ax, 'on'); 
 
% Filter out potential nodata values for UI limits calculation 
valid_mask_ui = x > -9000 & y > -9000; 
x_clean = x(valid_mask_ui); 
y_clean = y(valid_mask_ui); 
z_clean = z(valid_mask_ui); 
 
if isempty(x_clean) 
    x_clean = x; y_clean = y; z_clean = z; % Fallback 
end 
 
% Initialize plane parameters 
centroid = [mean(x_clean), mean(y_clean), mean(z_clean)]; 
planePos = centroid; 
rotationAngle = 0; % Rotation around Z-axis in degrees 
planeDistance = initialDistance; % Editable distance
targetAxis = 'Minimal'; % Default vertical alignment axis
if ~isempty(initialState)
    planePos = initialState.position;
    rotationAngle = initialState.rotationAngle;
    planeDistance = initialState.distance;
    targetAxis = initialState.targetAxis;
end

% Calculate plane size for visualization 
rangeX = max(x_clean) - min(x_clean); 
rangeY = max(y_clean) - min(y_clean); 
planeSize = max(rangeX, rangeY) * 0.8; 
 
% --- Create UI controls --- 
panel_pos_y_start = 750; panel_x_start = 1120; 
 
% Rotation control 
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start 200 20], ... 
    'String', 'Plane Rotation (Z-axis)', 'FontSize', 10, 'FontWeight', 'bold'); 
rotSlider = uicontrol('Style', 'slider', 'Position', [panel_x_start panel_pos_y_start-30 200 20], ... 
    'Min', -180, 'Max', 180, 'Value', rotationAngle, ...
    'Callback', @updatePlaneRotation);
rotEdit = uicontrol('Style', 'edit', 'Position', [panel_x_start+205 panel_pos_y_start-32 60 25], ...
    'String', sprintf('%.1f', rotationAngle), 'FontSize', 9, ...
    'TooltipString', 'Plane rotation in degrees (-180 to 180)', ...
    'Callback', @updateRotationFromEdit);
rotText = uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-55 200 20], ...
    'String', sprintf('Angle: %.1f°', rotationAngle), 'FontSize', 9);
 
% Distance control 
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-90 200 20], ... 
    'String', 'Distance Between Planes (in meters)', 'FontSize', 10, 'FontWeight', 'bold'); 
maxRange = max(rangeX, rangeY); 
distSlider = uicontrol('Style', 'slider', 'Position', [panel_x_start panel_pos_y_start-120 200 20], ... 
    'Min', 1, 'Max', max(maxRange, planeDistance), 'Value', planeDistance, ... 
    'Callback', @updatePlaneDistance); 
distText = uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-145 200 20], ... 
    'String', sprintf('Distance: %.1f m', planeDistance), 'FontSize', 9); 
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-175 100 20], ... 
    'String', 'Precise value:', 'FontSize', 9, 'HorizontalAlignment', 'left'); 
distEdit = uicontrol('Style', 'edit', 'Position', [panel_x_start+100 panel_pos_y_start-175 100 25], ... 
    'String', sprintf('%.1f', planeDistance), 'FontSize', 9, ... 
    'Callback', @updateDistanceFromEdit); 
 
% Plane Position controls 
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-225 200 20], ... 
    'String', 'Plane Position (Center)', 'FontSize', 10, 'FontWeight', 'bold'); 
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-255 20 20], 'String', 'X:', 'FontSize', 9, 'HorizontalAlignment', 'left'); 
posEditX = uicontrol('Style', 'edit', 'Position', [panel_x_start+25 panel_pos_y_start-255 100 25], ... 
    'String', sprintf('%.2f', planePos(1)), 'FontSize', 9, 'Callback', @updatePositionFromEdit); 
posSliderX = uicontrol('Style', 'slider', 'Position', [panel_x_start+130 panel_pos_y_start-255 100 20], ... 
    'Min', min(x_clean), 'Max', max(x_clean), 'Value', min(max(planePos(1), min(x_clean)), max(x_clean)), ... 
    'SliderStep', [0.0001 0.001], 'Callback', @updatePositionFromSliderX); 
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-285 20 20], 'String', 'Y:', 'FontSize', 9, 'HorizontalAlignment', 'left'); 
posEditY = uicontrol('Style', 'edit', 'Position', [panel_x_start+25 panel_pos_y_start-285 100 25], ... 
    'String', sprintf('%.2f', planePos(2)), 'FontSize', 9, 'Callback', @updatePositionFromEdit); 
posSliderY = uicontrol('Style', 'slider', 'Position', [panel_x_start+130 panel_pos_y_start-285 100 20], ... 
    'Min', min(y_clean), 'Max', max(y_clean), 'Value', min(max(planePos(2), min(y_clean)), max(y_clean)), ... 
    'SliderStep', [0.0001 0.001], 'Callback', @updatePositionFromSliderY); 
 
% Alignment Axis Selection 
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-370 200 20], ... 
    'String', 'Align Vertical With:', 'FontSize', 10, 'FontWeight', 'bold'); 
axisGroup = uibuttongroup('Parent', fig, 'Units', 'pixels', 'Position', [panel_x_start panel_pos_y_start-415 250 40], ... 
    'BorderType', 'none', 'SelectionChangedFcn', @updateAxisSelection); 
axisRadio.Y = uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'Y-axis', 'Position', [10 5 60 25], 'Value', strcmp(targetAxis, 'Y'));
axisRadio.X = uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'X-axis', 'Position', [70 5 60 25], 'Value', strcmp(targetAxis, 'X'));
axisRadio.Minimal = uicontrol(axisGroup, 'Style', 'radiobutton', 'String', 'Minimal', 'Position', [130 5 70 25], 'Value', strcmp(targetAxis, 'Minimal'));
 
% Point count display 
pointCountText = uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-475 270 60], ... 
    'String', '', 'FontSize', 9, 'HorizontalAlignment', 'left'); 
 
% Confirm button 
uicontrol('Style', 'pushbutton', 'Position', [panel_x_start panel_pos_y_start-530 200 40], ... 
    'String', 'Confirm Selection', 'FontSize', 12, 'Callback', @confirmSelection);

% Load the planes confirmed in the previous run
uicontrol('Style', 'pushbutton', 'Position', [panel_x_start panel_pos_y_start-580 200 35], ...
    'String', 'Load Last Planes', 'FontSize', 10, ...
    'TooltipString', 'Restore position, rotation, distance and axis used last time', ...
    'Callback', @loadLastPlanes);

% Instructions
uicontrol('Style', 'text', 'Position', [panel_x_start panel_pos_y_start-680 200 80], ...
    'String', 'Instructions: Drag arrows to move plane. Use sliders or enter values to adjust rotation, distance, and position. Select alignment axis.', ... 
    'FontSize', 9, 'HorizontalAlignment', 'left'); 
 
% Storage for graphics objects 
planeHandles = struct(); 
arrowHandles = struct(); 
 
% Draw initial planes 
updatePlaneVisualization(); 
 
% Wait for user confirmation 
addVerticalScroll(fig);
uiwait(fig); 
 
% Return final values 
finalDistance = planeDistance; 
planePosition = planePos; 
planeNormal = calculatePlaneNormal();
planeState = struct('position', planePos, 'rotationAngle', rotationAngle, ...
    'distance', planeDistance, 'targetAxis', targetAxis);

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
        set(rotEdit, 'String', sprintf('%.1f', rotationAngle));
        updatePlaneVisualization();
    end

    function updateRotationFromEdit(src, ~)
        newAngle = str2double(get(src, 'String'));
        if isnan(newAngle)
            set(src, 'String', sprintf('%.1f', rotationAngle));
            return;
        end
        % Wrap into the slider range [-180, 180]
        rotationAngle = mod(newAngle + 180, 360) - 180;
        set(src, 'String', sprintf('%.1f', rotationAngle));
        set(rotSlider, 'Value', rotationAngle);
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
            set(src, 'String', sprintf('%.1f', planeDistance));
        end          
    end          
  
    function updatePositionFromEdit(~, ~)          
        newX = str2double(get(posEditX, 'String'));          
        newY = str2double(get(posEditY, 'String'));          
        if ~isnan(newX), planePos(1) = newX; end          
        if ~isnan(newY), planePos(2) = newY; end          
        updatePlaneVisualization();          
    end          
 
    function updatePositionFromSliderX(src, ~)     
        val = get(src, 'Value');     
        planePos(1) = val;     
        updatePlaneVisualization();     
    end     
 
    function updatePositionFromSliderY(src, ~)     
        val = get(src, 'Value');     
        planePos(2) = val;     
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
 
        % Update sliders     
        if planePos(1) >= get(posSliderX, 'Min') && planePos(1) <= get(posSliderX, 'Max')     
            set(posSliderX, 'Value', planePos(1));     
        end     
        if planePos(2) >= get(posSliderY, 'Min') && planePos(2) <= get(posSliderY, 'Max')     
            set(posSliderY, 'Value', planePos(2));     
        end     
  
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
        planePts1 = planePts1 + (rand(size(planePts1))-0.5)*0.001;     
        planeHandles.plane1 = surf(ax, reshape(planePts1(:,1), size(u)), reshape(planePts1(:,2), size(u)), reshape(planePts1(:,3), size(u)), ...          
                                   'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');          
        planePos2 = planePos + planeDistance * normal;          
        planePts2 = planePos2 + u(:)*v1 + v(:)*v2;          
        planePts2 = planePts2 + (rand(size(planePts2))-0.5)*0.001;     
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
        uiresume(fig);
        close(fig);
    end

    function loadLastPlanes(~, ~)
        if isempty(lastPlaneFile) || ~isfile(lastPlaneFile)
            warning('No saved planes found (%s). Confirm a plane once to create it.', lastPlaneFile);
            warndlg('No saved planes found. Confirm a plane once to create them.', 'Load Last Planes');
            return;
        end
        try
            S = load(lastPlaneFile, 'planeState');
            st = S.planeState;
            planePos = st.position;
            rotationAngle = st.rotationAngle;
            planeDistance = st.distance;
            targetAxis = st.targetAxis;
        catch ME
            warning('Could not read %s: %s', lastPlaneFile, ME.message);
            warndlg('The saved planes file could not be read.', 'Load Last Planes');
            return;
        end

        % Rotation controls
        set(rotSlider, 'Value', rotationAngle);
        set(rotEdit, 'String', sprintf('%.1f', rotationAngle));
        set(rotText, 'String', sprintf('Angle: %.1f°', rotationAngle));

        % Distance controls (extend slider range if needed)
        if planeDistance > get(distSlider, 'Max'), set(distSlider, 'Max', planeDistance * 1.2); end
        set(distSlider, 'Value', max(planeDistance, get(distSlider, 'Min')));
        set(distText, 'String', sprintf('Distance: %.1f m', planeDistance));
        set(distEdit, 'String', sprintf('%.1f', planeDistance));

        % Alignment axis
        if isfield(axisRadio, targetAxis)
            set(axisGroup, 'SelectedObject', axisRadio.(targetAxis));
        end

        % Position edits/sliders and planes are refreshed here
        updatePlaneVisualization();
        fprintf('Loaded last planes from %s\n', lastPlaneFile);
    end
end
 
%% FILTER BY EDGE LENGTH
function [faces_t] = filter_by_length(T_filtered, x_filtered, y_filtered, max_edge_length) 
% Filter triangles by edge length 
num_triangles = size(T_filtered, 1); 
valid_triangles = true(num_triangles, 1); 
 
% Validate inputs
if isempty(T_filtered) || isempty(x_filtered) || isempty(y_filtered)
    faces_t = [];
    return;
end

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
 
%% INTERACTIVE ROTATION SELECTOR
function [tform_manual, goBack] = interactiveRotationSelector(ptCloud)
% goBack = true when the user asks to return to the plane selection
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
rotationAngle = 0; % Default angle in degrees 
tform_manual = affinetform3d(eye(4)); % Default transform (identity)
goBack = false;
 
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

% Back button: discard this cut and choose the plane again
uicontrol('Style', 'pushbutton', 'Position', [750 panel_pos_y_start-350 200 40], ...
    'String', '< Back to Plane Selection', 'FontSize', 10, 'Callback', @backToPlaneSelection);

% Storage for graphics objects
axisHandle = [];
 
% Initial visualization 
updateVisualization(); 
 
% Wait for user to confirm 
addVerticalScroll(fig);
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
            set(src, 'String', num2str(rotationAngle));
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
        affine_matrix_manual(1:3, 1:3) = R_manual; % affinetform3d uses column vectors
                  
        % Check if translation is needed          
        if get(translateCheckbox, 'Value') == 1          
            % Create a temporary transform for rotation only          
            tform_rot_only = affinetform3d(affine_matrix_manual);          
            % Rotate the point cloud to find its new minimum Z          
            rotated_pts = transformPointsForward(tform_rot_only, ptCloud.Location);          
            minZ = min(rotated_pts(:,3));          
            % Add translation to the matrix          
            translation_vec = [0, 0, -minZ];          
            affine_matrix_manual(1:3, 4) = translation_vec(:);          
        end          
                  
        % Create the final combined transformation          
        tform_manual = affinetform3d(affine_matrix_manual);

        uiresume(fig);
        close(fig);
    end

    function backToPlaneSelection(~, ~)
        goBack = true;
        uiresume(fig);
        close(fig);
    end
end
 
%% UTILITY
function s = choose_str(condition, a, b)
    if condition, s = a; else, s = b; end
end

%% POISSON RECONSTRUCTION RUNNER
function [vertices, faces, success] = runPoissonRecon(pts_xyz, folder, name, exePath, depth, pointWeight, normalNeighbors, enableTrimmer, trimValue)
% RUNPOISSONRECON  Call mkazhdan/PoissonRecon (+ optional SurfaceTrimmer).
%   1. Estimates oriented normals
%   2. Writes ASCII PLY with normals
%   3. Runs PoissonRecon.exe (with --density for trimming support)
%   4. Optionally runs SurfaceTrimmer.exe to remove low-density artifacts
%   5. Reads the final mesh back into MATLAB
%   Returns success=false on any failure so the caller falls back to Delaunay.

if nargin < 8, enableTrimmer = false; end
if nargin < 9, trimValue = 7; end

vertices = [];
faces    = [];
success  = false;

% ---- 1. Estimate normals ------------------------------------------------
fprintf('  Estimating normals (%d neighbours)...\n', normalNeighbors);
try
    ptc = pointCloud(pts_xyz);
    normals = pcnormals(ptc, normalNeighbors);
catch ME
    warning('Normal estimation failed: %s', ME.message);
    return;
end

% Orient normals consistently away from the point cloud centroid.
% This handles vertical faces and overhangs correctly for full 3D reconstruction,
% unlike a simple +Z flip which only works for flat terrain.
centroid_pts = mean(pts_xyz, 1);
outward_vecs = pts_xyz - centroid_pts;  % vector from centroid to each point
flip_mask = dot(normals, outward_vecs, 2) < 0;
normals(flip_mask, :) = -normals(flip_mask, :);

% ---- 2. Write input PLY with normals ------------------------------------
ply_in  = fullfile(folder, name + "_poisson_in.ply");
ply_raw = fullfile(folder, name + "_poisson_raw.ply");

fprintf('  Writing input PLY: %s\n', ply_in);
try
    writePLYWithNormals(pts_xyz, normals, ply_in);
catch ME
    warning('Failed to write input PLY: %s', ME.message);
    return;
end

% ---- 3. Run PoissonRecon ------------------------------------------------
% --density  : embeds per-vertex density in output PLY (required by SurfaceTrimmer)
densityFlag = '';
if enableTrimmer
    densityFlag = ' --density';
end
cmd = sprintf('"%s" --in "%s" --out "%s" --depth %d --pointWeight %.4f%s', ...
    exePath, ply_in, ply_raw, depth, pointWeight, densityFlag);
fprintf('  Running PoissonRecon...\n  Command: %s\n', cmd);
[status, cmdout] = system(cmd);

if status ~= 0
    warning('PoissonRecon returned status %d:\n%s', status, cmdout);
    return;
end
if ~exist(ply_raw, 'file')
    warning('PoissonRecon output file not found: %s', ply_raw);
    return;
end
fprintf('  PoissonRecon done.\n');

% ---- 4. Run SurfaceTrimmer (optional) -----------------------------------
ply_out = ply_raw;   % default: use raw output

if enableTrimmer
    trimmerPath = fullfile(fileparts(exePath), 'SurfaceTrimmer.exe');
    if exist(trimmerPath, 'file')
        ply_trimmed = fullfile(folder, name + "_poisson_trimmed.ply");
        cmd_trim = sprintf('"%s" --in "%s" --out "%s" --trim %.2f', ...
            trimmerPath, ply_raw, ply_trimmed, trimValue);
        fprintf('  Running SurfaceTrimmer (trim=%.2f)...\n  Command: %s\n', trimValue, cmd_trim);
        [status_t, cmdout_t] = system(cmd_trim);
        if status_t == 0 && exist(ply_trimmed, 'file')
            ply_out = ply_trimmed;
            fprintf('  SurfaceTrimmer done — artifacts removed.\n');
        else
            warning('SurfaceTrimmer failed (status %d). Using untrimmed mesh.\n%s', status_t, cmdout_t);
        end
    else
        warning('SurfaceTrimmer not found at: %s\nUsing untrimmed mesh.', trimmerPath);
    end
end

% ---- 5. Read final mesh --------------------------------------------------
fprintf('  Reading mesh: %s\n', ply_out);
try
    [vertices, faces] = readPLYMesh(ply_out);
catch ME
    warning('Failed to read mesh: %s', ME.message);
    return;
end

if isempty(vertices) || isempty(faces)
    warning('Output PLY contains no mesh data.');
    return;
end

fprintf('  Mesh loaded: %d vertices, %d faces\n', size(vertices,1), size(faces,1));
success = true;
end

%% WRITE PLY WITH NORMALS (ASCII)
function writePLYWithNormals(pts, normals, filepath)
% Write an ASCII PLY file containing XYZ + normal NX NY NZ.
% Required input format for PoissonRecon.

n   = size(pts, 1);
fid = fopen(filepath, 'w');
if fid == -1
    error('Cannot open file for writing: %s', filepath);
end
fprintf(fid, 'ply\n');
fprintf(fid, 'format ascii 1.0\n');
fprintf(fid, 'element vertex %d\n', n);
fprintf(fid, 'property float x\nproperty float y\nproperty float z\n');
fprintf(fid, 'property float nx\nproperty float ny\nproperty float nz\n');
fprintf(fid, 'end_header\n');

% Write all data in one vectorised call for speed
data = [pts, normals]';          % 6 x N
fprintf(fid, '%f %f %f %f %f %f\n', data);
fclose(fid);
end

%% READ PLY MESH  (handles ASCII and binary little-endian)
function [vertices, faces] = readPLYMesh(filepath)
% READPLYMESH  Read a triangle mesh from a PLY file.
%   Supports both ASCII and binary-little-endian formats as produced by
%   PoissonRecon.  Falls back to readSurfaceMesh (R2021a+) when available.

vertices = [];
faces    = [];

% --- Try MATLAB's readSurfaceMesh first (R2021a+, handles binary PLY) ---
if exist('readSurfaceMesh', 'file')
    try
        sm       = readSurfaceMesh(filepath);
        vertices = double(sm.Vertices);
        faces    = double(sm.Faces);
        return;
    catch
        % fall through to manual reader
    end
end

% --- Manual reader -------------------------------------------------------
fid = fopen(filepath, 'rb');
if fid == -1
    error('Cannot open PLY file: %s', filepath);
end

numVertices  = 0;
numFaces     = 0;
isBinaryLE   = false;
isBinaryBE   = false;
numVtxProps  = 0;   % total float/double vertex properties before nx/ny/nz

while true
    line = strtrim(fgetl(fid));
    if ~ischar(line), break; end
    if startsWith(line, 'format binary_little_endian'), isBinaryLE = true; end
    if startsWith(line, 'format binary_big_endian'),    isBinaryBE = true; end
    if startsWith(line, 'element vertex')
        tok = strsplit(line);
        numVertices = str2double(tok{3});
    end
    if startsWith(line, 'element face')
        tok = strsplit(line);
        numFaces = str2double(tok{3});
    end
    if startsWith(line, 'property') && numFaces == 0
        numVtxProps = numVtxProps + 1;  % count all vertex props
    end
    if strcmp(line, 'end_header'), break; end
end

if isBinaryLE || isBinaryBE
    % Binary path — all vertex properties assumed float32 (standard for PoissonRecon).
    % Read all numVtxProps floats per vertex, keep only the first 3 (x y z).
    vertices = zeros(numVertices, 3, 'single');
    for i = 1:numVertices
        raw = fread(fid, numVtxProps, 'float32=>float32');
        if isBinaryBE
            raw = swapbytes(raw);
        end
        vertices(i,:) = raw(1:3)';
    end
    vertices = double(vertices);

    % Faces: each face = uchar count + count*int32 indices
    faces = zeros(numFaces, 3, 'int32');
    for i = 1:numFaces
        cnt  = fread(fid, 1, 'uint8');
        idx  = fread(fid, cnt, 'int32=>int32');
        if isBinaryBE, idx = swapbytes(idx); end
        faces(i,:) = idx(1:3)';
    end
    faces = double(faces) + 1;  % PLY is 0-indexed → 1-indexed
else
    % ASCII path
    vertices = zeros(numVertices, 3);
    for i = 1:numVertices
        tok = strsplit(strtrim(fgetl(fid)));
        vertices(i,:) = [str2double(tok{1}), str2double(tok{2}), str2double(tok{3})];
    end
    faces = zeros(numFaces, 3);
    for i = 1:numFaces
        tok = strsplit(strtrim(fgetl(fid)));
        % tok{1} = face vertex count
        faces(i,:) = [str2double(tok{2}), str2double(tok{3}), str2double(tok{4})];
        faces(i,:) = faces(i,:) + 1;  % 0-indexed → 1-indexed
    end
end

fclose(fid);
end

%% REMOVE ISOLATED MESH COMPONENTS
function [vertices, faces] = removeIsolatedComponents(vertices, faces)
% Keep only the largest connected face component.
% Two faces are connected if they share at least one vertex.

n_faces = size(faces, 1);
n_verts = size(vertices, 1);
if n_faces == 0, return; end

% Build vertex-to-face adjacency list
vtx2face = cell(n_verts, 1);
for f = 1:n_faces
    for k = 1:3
        vtx2face{faces(f,k)}(end+1) = f;
    end
end

% BFS over faces to label connected components
component = zeros(n_faces, 1);
comp_id = 0;
for seed = 1:n_faces
    if component(seed) > 0, continue; end
    comp_id = comp_id + 1;
    queue = seed;
    head  = 1;
    component(seed) = comp_id;
    while head <= numel(queue)
        f = queue(head); head = head + 1;
        for k = 1:3
            for fn = vtx2face{faces(f,k)}
                if component(fn) == 0
                    component(fn) = comp_id;
                    queue(end+1) = fn; %#ok<AGROW>
                end
            end
        end
    end
end

% Keep the largest component
comp_sizes = accumarray(component, 1, [comp_id, 1]);
[~, largest] = max(comp_sizes);
keep_mask = component == largest;
fprintf('    Components found: %d — keeping largest (%d / %d faces)\n', ...
    comp_id, sum(keep_mask), n_faces);
faces = faces(keep_mask, :);

% Remove unused vertices and reindex faces
[used, ~, new_idx] = unique(faces(:));
vertices = vertices(used, :);
faces = reshape(new_idx, [], 3);
end

%% CLOSE OPEN MESH WITH FLAT BOTTOM
function [vertices_out, faces_out] = closeMeshWithFlatBottom(vertices, faces, z_flat)
% If the mesh has open boundary edges, project them down to z_flat,
% add side walls, and cap the bottom with a 2D Delaunay triangulation.

TR_tmp = triangulation(faces, vertices);
bndry  = freeBoundary(TR_tmp);   % Mx2 ordered boundary edges

if isempty(bndry)
    fprintf('    Mesh is already closed — no bottom needed.\n');
    vertices_out = vertices;
    faces_out    = faces;
    return;
end

fprintf('    Boundary edges: %d\n', size(bndry,1));

% Chain boundary edges into an ordered vertex loop
loop_verts = chainEdges(bndry);
n_loop = numel(loop_verts);

if n_loop < 3
    warning('MESH:bottomFail', 'Could not order boundary loop — skipping bottom.');
    vertices_out = vertices;
    faces_out    = faces;
    return;
end

% Bottom ring: project loop vertices to z_flat
n_orig  = size(vertices, 1);
bot_xyz = [vertices(loop_verts, 1:2), repmat(z_flat, n_loop, 1)];
bot_idx = (n_orig+1 : n_orig+n_loop);

vertices_out = [vertices; bot_xyz];
faces_out    = faces;

% Side walls connecting top boundary to bottom ring
side_faces = zeros(2*n_loop, 3);
for i = 1:n_loop
    j   = mod(i, n_loop) + 1;
    v1t = loop_verts(i);  v2t = loop_verts(j);
    v1b = bot_idx(i);     v2b = bot_idx(j);
    side_faces(2*i-1, :) = [v1t, v2t, v2b];
    side_faces(2*i,   :) = [v1t, v2b, v1b];
end
faces_out = [faces_out; side_faces];

% Bottom cap — 2D Delaunay of the bottom ring for robustness
try
    DT_bot = delaunayTriangulation(bot_xyz(:,1), bot_xyz(:,2));
    cap    = DT_bot.ConnectivityList;           % local indices 1..n_loop
    cap_global = bot_idx(cap);                  % map to global indices
    cap_global = fliplr(cap_global);            % flip winding to face downward
    faces_out = [faces_out; cap_global];
    fprintf('    Bottom cap: %d triangles\n', size(cap,1));
catch ME
    warning('MESH:bottomCap', 'Bottom cap triangulation failed: %s', ME.message);
end

fprintf('    Closed mesh: %d vertices, %d faces\n', ...
    size(vertices_out,1), size(faces_out,1));
end

%% CHAIN BOUNDARY EDGES INTO ORDERED LOOP
function loop = chainEdges(edges)
% Given an Mx2 array of boundary edges [from, to], return an ordered
% vertex sequence forming a single loop.

n = size(edges, 1);
[from_s, sort_idx] = sort(edges(:,1));
to_s = edges(sort_idx, 2);

start   = from_s(1);
loop    = start;
current = to_s(1);
iter    = 0;
while current ~= start && iter < n
    iter = iter + 1;
    loop(end+1) = current; %#ok<AGROW>
    pos = find(from_s == current, 1);
    if isempty(pos)
        warning('MESH:openEdgeChain', ...
            'Boundary edge chain is broken — mesh may have multiple open loops. Bottom cap may be incomplete.');
        break;
    end
    current = to_s(pos);
end
end

%% MESH PEAK FILTERING
function [newVertices, newFaces, spikeVertices] = filterMeshPeaks(vertices, faces, spikeAngle, heightThreshold) 
% FILTERMESHPEAKS Remove peak- or pyramid-like protrusions from a 3D mesh 
 
if nargin < 3, spikeAngle = 45; end 
if nargin < 4, heightThreshold = 0.01; end 
 
nV = size(vertices,1); 
nF = size(faces,1); 
 
%% --- Compute face normals --- 
p1 = vertices(faces(:,1),:); 
p2 = vertices(faces(:,2),:); 
p3 = vertices(faces(:,3),:); 
fn = cross(p2 - p1, p3 - p1, 2); 
fn = fn ./ max(vecnorm(fn,2,2), eps); % normalize 
 
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
 
%% ADVANCED MESH WELDING
function [newVertices, faces, modifiedIndices, stats] = advancedMeshWelding(vertices, faces, weldingAngle, coplanarAngle, maxIterations, peakThreshold) 
% ADVANCEDMESHWELDING Filter vertices using interior and dihedral angles 
 
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
 
%% HELPER FUNCTIONS FOR MESH WELDING
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
 
%% METHOD SELECTION UI
function method = methodSelectionUI()
method = '';
fig = figure('Name', 'Select Meshing Method', 'NumberTitle', 'off', ...
    'Position', [400 400 420 175], 'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off');

uicontrol('Style', 'text', 'Position', [20 135 380 28], ...
    'String', 'Select Meshing Method', 'FontSize', 13, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

bg = uibuttongroup('Parent', fig, 'Units', 'pixels', 'Position', [20 60 380 68], ...
    'BorderType', 'none');
uicontrol(bg, 'Style', 'radiobutton', 'Position', [10 38 360 24], ...
    'String', 'PoissonRecon  —  full 3D reconstruction  (recommended for outcrops)', ...
    'FontSize', 10, 'Value', 1, 'Tag', 'Poisson');
uicontrol(bg, 'Style', 'radiobutton', 'Position', [10 8 360 24], ...
    'String', 'Delaunay  —  2.5D projection  (terrain / flat surfaces)', ...
    'FontSize', 10, 'Tag', 'Delaunay');

uicontrol('Style', 'pushbutton', 'Position', [135 15 150 35], ...
    'String', 'Continue', 'FontSize', 11, 'Callback', @confirm);

addVerticalScroll(fig);
uiwait(fig);
if ishandle(fig), delete(fig); end

    function confirm(~,~)
        method = bg.SelectedObject.Tag;
        uiresume(fig);
    end
end

%% POISSON PARAMETER UI
function params = poissonParameterUI()

% --- Default values (all fields populated to avoid missing-field errors) ---
params = struct();
params.gridStep             = 2;
params.max_edge_length      = 4;
params.z_deep               = 20;
params.planeDistance        = 50;
params.enableWelding        = false;
params.weldingAngle         = 40;
params.peakThreshold        = 0.005;
params.enableInterpolation  = false;
params.enableDownsampling   = false;
params.interpolationMethod  = 'nearest';
params.extrapolationMethod  = 'none';
params.refcycle             = 1;
params.skipAutoInverse      = false;
params.skipManualInverse    = false;
params.createSolidMesh      = false;  % off by default for Poisson — use 'Add flat bottom' checkbox
params.enableOverlapFilter  = false;
params.overlapRadius        = 0.5;
params.usePoissonRecon      = true;
params.poissonReconPath     = '';
params.poissonDepth         = 10;
params.poissonPointWeight   = 0;
params.poissonNormalNeighbors = 20;
params.enablePoissonTrimmer = true;
params.poissonTrimValue     = 7;
params.translateToOrigin    = false;
params.skipPlaneCutting     = false;
params.outlierNumNeighbors  = 8;
params.outlierThreshold     = 1.0;

% Auto-detect PoissonRecon.exe
scriptDir = fileparts(mfilename('fullpath'));
autoPath  = fullfile(scriptDir, 'PoissonRecon.exe');
if exist(autoPath, 'file')
    params.poissonReconPath = autoPath;
    fprintf('PoissonRecon.exe auto-detected at: %s\n', autoPath);
end

% --- UI ---
fig = figure('Name', 'PoissonRecon Parameters', 'NumberTitle', 'off', ...
    'Position', [300 50 500 730], 'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off');

h = 24; sp = 32; lw = 185; ew = 165; lx = 15; ex = 210;
y = 695;

% Outlier Removal
uicontrol('Style', 'text', 'Position', [lx y 470 22], 'String', '— Outlier Removal (pcdenoise) —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Num Neighbors:', 'HorizontalAlignment', 'right');
edit_outlierN = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '8');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Threshold (std dev):', 'HorizontalAlignment', 'right');
edit_outlierT = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '1.0');
uicontrol('Style', 'text', 'Position', [ex+ew+5 y 80 h], 'String', 'higher=keep more', 'FontSize', 8);

% PoissonRecon Settings
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 470 22], 'String', '— PoissonRecon Settings —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'PoissonRecon.exe:', 'HorizontalAlignment', 'right');
edit_poissonPath = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', params.poissonReconPath);
uicontrol('Style', 'pushbutton', 'Position', [ex+ew+5 y 42 h], 'String', '...', 'Callback', @browsePoissonExe);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Depth (8–12):', 'HorizontalAlignment', 'right');
edit_depth = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '10');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Point Weight (0=interpolate):', 'HorizontalAlignment', 'right');
edit_weight = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '0');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Normal Neighbors:', 'HorizontalAlignment', 'right');
edit_normalN = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '20');
y = y - sp;
chk_trimmer = uicontrol('Style', 'checkbox', 'Position', [ex y ew+60 h], ...
    'String', 'Run SurfaceTrimmer', 'Value', 1, 'Callback', @toggleTrimmer);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Trim Value (5–10):', 'HorizontalAlignment', 'right');
edit_trim = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '7');
uicontrol('Style', 'text', 'Position', [ex+ew+5 y 85 h], 'String', 'low=keep more', 'FontSize', 8);

% Mesh Settings
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 470 22], 'String', '— Mesh Settings —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Max Edge Length:', 'HorizontalAlignment', 'right');
edit_edge = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '4');
y = y - sp;
chk_solid = uicontrol('Style', 'checkbox', 'Position', [ex y ew+160 h], ...
    'String', 'Add flat bottom (close open boundary)', 'Value', 0, 'Callback', @toggleSolid);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Z Deep:', 'HorizontalAlignment', 'right');
edit_zdeep = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '20', 'Enable', 'off');

% Plane & Coordinate Settings
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 470 22], 'String', '— Plane & Coordinate Settings —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Plane Distance (m):', 'HorizontalAlignment', 'right');
edit_planeDist = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '50');
y = y - sp;
chk_skipCutting   = uicontrol('Style', 'checkbox', 'Position', [ex y ew+100 h], 'String', 'Skip Plane Cutting', 'Value', 0);
y = y - sp;
chk_transToOrigin = uicontrol('Style', 'checkbox', 'Position', [ex y ew+150 h], 'String', 'Translate to Origin after Alignment', 'Value', 0);
y = y - sp;
chk_skipManualInv = uicontrol('Style', 'checkbox', 'Position', [ex y ew+150 h], 'String', 'Skip Manual Inverse Rotation', 'Value', 0);
y = y - sp;
chk_skipAutoInv   = uicontrol('Style', 'checkbox', 'Position', [ex y ew+150 h], 'String', 'Skip Automatic Inverse Rotation', 'Value', 0);

% Start button
y = y - sp - 4;
uicontrol('Style', 'pushbutton', 'Position', [150 y 200 38], 'String', 'Start Processing', ...
    'FontSize', 12, 'Callback', @startProcessing);

addVerticalScroll(fig);
uiwait(fig);

    function toggleTrimmer(src,~)
        if get(src,'Value'), set(edit_trim,'Enable','on');
        else, set(edit_trim,'Enable','off'); end
    end
    function toggleSolid(src,~)
        if get(src,'Value'), set(edit_zdeep,'Enable','on');
        else, set(edit_zdeep,'Enable','off'); end
    end
    function browsePoissonExe(~,~)
        [f,p] = uigetfile({'PoissonRecon.exe;*.exe','Executable (*.exe)';'*.*','All Files'}, 'Select PoissonRecon.exe');
        if f ~= 0, set(edit_poissonPath,'String',fullfile(p,f)); end
    end
    function startProcessing(~,~)
        params.outlierNumNeighbors    = str2double(get(edit_outlierN,    'String'));
        params.outlierThreshold       = str2double(get(edit_outlierT,    'String'));
        params.poissonReconPath       = strtrim(get(edit_poissonPath,    'String'));
        params.poissonDepth           = str2double(get(edit_depth,       'String'));
        params.poissonPointWeight     = str2double(get(edit_weight,      'String'));
        params.poissonNormalNeighbors = str2double(get(edit_normalN,     'String'));
        params.enablePoissonTrimmer   = get(chk_trimmer,  'Value');
        params.poissonTrimValue       = str2double(get(edit_trim,        'String'));
        params.max_edge_length        = str2double(get(edit_edge,        'String'));
        params.createSolidMesh        = get(chk_solid,    'Value');
        params.z_deep                 = str2double(get(edit_zdeep,       'String'));
        params.planeDistance          = str2double(get(edit_planeDist,   'String'));
        params.skipPlaneCutting       = get(chk_skipCutting,   'Value');
        params.translateToOrigin      = get(chk_transToOrigin, 'Value');
        params.skipManualInverse      = get(chk_skipManualInv, 'Value');
        params.skipAutoInverse        = get(chk_skipAutoInv,   'Value');
        uiresume(fig);
        close(fig);
    end
end

%% DELAUNAY PARAMETER UI
function params = delaunayParameterUI()

% --- Default values ---
params = struct();
params.gridStep             = 2;
params.max_edge_length      = 4;
params.z_deep               = 20;
params.planeDistance        = 50;
params.enableWelding        = false;
params.weldingAngle         = 40;
params.peakThreshold        = 0.005;
params.enableInterpolation  = true;
params.enableDownsampling   = true;
params.interpolationMethod  = 'nearest';
params.extrapolationMethod  = 'none';
params.refcycle             = 1;
params.skipAutoInverse      = false;
params.skipManualInverse    = false;
params.createSolidMesh      = true;
params.enableOverlapFilter  = false;
params.overlapRadius        = 0.5;
params.usePoissonRecon      = false;
params.poissonReconPath     = '';
params.poissonDepth         = 10;
params.poissonPointWeight   = 0;
params.poissonNormalNeighbors = 20;
params.enablePoissonTrimmer = false;
params.poissonTrimValue     = 7;
params.translateToOrigin    = false;
params.skipPlaneCutting     = false;
params.outlierNumNeighbors  = 8;
params.outlierThreshold     = 1.0;

% --- UI ---
fig = figure('Name', 'Delaunay Parameters', 'NumberTitle', 'off', ...
    'Position', [300 50 520 990], 'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off');

h = 24; sp = 32; lw = 200; ew = 165; lx = 15; ex = 225;
y = 955;

% Mesh Settings
uicontrol('Style', 'text', 'Position', [lx y 490 22], 'String', '— Mesh Settings —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Grid Step:', 'HorizontalAlignment', 'right');
edit_gridStep = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '2');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Max Edge Length:', 'HorizontalAlignment', 'right');
edit_edge = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '4');
y = y - sp;
chk_solid = uicontrol('Style', 'checkbox', 'Position', [ex y ew+80 h], ...
    'String', 'Create Solid Block (Extrude)', 'Value', 1, 'Callback', @toggleSolid);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Z Deep:', 'HorizontalAlignment', 'right');
edit_zdeep = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '20');

% Pre-processing
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 490 22], 'String', '— Pre-processing —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
chk_downsample = uicontrol('Style', 'checkbox', 'Position', [ex y ew+100 h], ...
    'String', 'Enable Downsampling & Denoising', 'Value', 1);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Outlier Num Neighbors:', 'HorizontalAlignment', 'right');
edit_outlierN = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '8');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Outlier Threshold (std dev):', 'HorizontalAlignment', 'right');
edit_outlierT = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '1.0');
uicontrol('Style', 'text', 'Position', [ex+ew+5 y 80 h], 'String', 'higher=keep more', 'FontSize', 8);

% Overlap Filter
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 490 22], 'String', '— Overlap Filter —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
chk_overlap = uicontrol('Style', 'checkbox', 'Position', [ex y ew+100 h], ...
    'String', 'Enable Overlap Filter (Keep Lowest Z)', 'Value', 0, 'Callback', @toggleOverlap);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Overlap Radius (m):', 'HorizontalAlignment', 'right');
edit_overlapRad = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '0.5', 'Enable', 'off');

% Mesh Welding
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 490 22], 'String', '— Mesh Welding —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
chk_welding = uicontrol('Style', 'checkbox', 'Position', [ex y ew+80 h], ...
    'String', 'Enable Mesh Welding', 'Value', 0, 'Callback', @toggleWelding);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Welding Angle:', 'HorizontalAlignment', 'right');
edit_weldAngle = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '40', 'Enable', 'off');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Peak Threshold:', 'HorizontalAlignment', 'right');
edit_peakThresh = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '0.005', 'Enable', 'off');

% Interpolation / Resampling
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 490 22], 'String', '— Interpolation / Resampling —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
chk_interp = uicontrol('Style', 'checkbox', 'Position', [ex y ew+80 h], ...
    'String', 'Enable Resampling', 'Value', 1, 'Callback', @toggleInterp);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Interpolation Method:', 'HorizontalAlignment', 'right');
popup_interp = uicontrol('Style', 'popupmenu', 'Position', [ex y ew h], ...
    'String', {'linear','nearest','natural'}, 'Value', 2);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Extrapolation Method:', 'HorizontalAlignment', 'right');
popup_extrap = uicontrol('Style', 'popupmenu', 'Position', [ex y ew h], ...
    'String', {'linear','nearest','none'}, 'Value', 3);
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Refinement Cycles:', 'HorizontalAlignment', 'right');
edit_refcycle = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '1');

% Plane & Coordinate Settings
y = y - sp - 6;
uicontrol('Style', 'text', 'Position', [lx y 490 22], 'String', '— Plane & Coordinate Settings —', ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - sp;
uicontrol('Style', 'text', 'Position', [lx y lw h], 'String', 'Plane Distance (m):', 'HorizontalAlignment', 'right');
edit_planeDist = uicontrol('Style', 'edit', 'Position', [ex y ew h], 'String', '50');
y = y - sp;
chk_skipCutting   = uicontrol('Style', 'checkbox', 'Position', [ex y ew+100 h], 'String', 'Skip Plane Cutting', 'Value', 0);
y = y - sp;
chk_transToOrigin = uicontrol('Style', 'checkbox', 'Position', [ex y ew+160 h], 'String', 'Translate to Origin after Alignment', 'Value', 0);
y = y - sp;
chk_skipManualInv = uicontrol('Style', 'checkbox', 'Position', [ex y ew+150 h], 'String', 'Skip Manual Inverse Rotation', 'Value', 0);
y = y - sp;
chk_skipAutoInv   = uicontrol('Style', 'checkbox', 'Position', [ex y ew+150 h], 'String', 'Skip Auto Inverse Rotation', 'Value', 0);

% Start button
y = y - sp - 4;
uicontrol('Style', 'pushbutton', 'Position', [160 y 200 38], 'String', 'Start Processing', ...
    'FontSize', 12, 'Callback', @startProcessing);

addVerticalScroll(fig);
uiwait(fig);

    function toggleSolid(src,~)
        if get(src,'Value'), set(edit_zdeep,'Enable','on');
        else, set(edit_zdeep,'Enable','off'); end
    end
    function toggleOverlap(src,~)
        if get(src,'Value'), set(edit_overlapRad,'Enable','on');
        else, set(edit_overlapRad,'Enable','off'); end
    end
    function toggleWelding(src,~)
        en = 'off'; if get(src,'Value'), en = 'on'; end
        set(edit_weldAngle, 'Enable', en);
        set(edit_peakThresh,'Enable', en);
    end
    function toggleInterp(src,~)
        en = 'off'; if get(src,'Value'), en = 'on'; end
        set(popup_interp,  'Enable', en);
        set(popup_extrap,  'Enable', en);
        set(edit_refcycle, 'Enable', en);
    end
    function startProcessing(~,~)
        params.gridStep            = str2double(get(edit_gridStep,  'String'));
        params.max_edge_length     = str2double(get(edit_edge,       'String'));
        params.createSolidMesh     = get(chk_solid,       'Value');
        params.z_deep              = str2double(get(edit_zdeep,      'String'));
        params.enableDownsampling  = get(chk_downsample,  'Value');
        params.outlierNumNeighbors = str2double(get(edit_outlierN,   'String'));
        params.outlierThreshold    = str2double(get(edit_outlierT,   'String'));
        params.enableOverlapFilter = get(chk_overlap,     'Value');
        params.overlapRadius       = str2double(get(edit_overlapRad, 'String'));
        params.enableWelding       = get(chk_welding,     'Value');
        params.weldingAngle        = str2double(get(edit_weldAngle,  'String'));
        params.peakThreshold       = str2double(get(edit_peakThresh, 'String'));
        params.enableInterpolation = get(chk_interp,      'Value');
        interpOpts = {'linear','nearest','natural'};
        extrapOpts = {'linear','nearest','none'};
        params.interpolationMethod = interpOpts{get(popup_interp, 'Value')};
        params.extrapolationMethod = extrapOpts{get(popup_extrap, 'Value')};
        params.refcycle            = str2double(get(edit_refcycle,   'String'));
        params.planeDistance       = str2double(get(edit_planeDist,  'String'));
        params.skipPlaneCutting    = get(chk_skipCutting,   'Value');
        params.translateToOrigin   = get(chk_transToOrigin, 'Value');
        params.skipManualInverse   = get(chk_skipManualInv, 'Value');
        params.skipAutoInverse     = get(chk_skipAutoInv,   'Value');
        uiresume(fig);
        close(fig);
    end
end

%% PARAMETER SELECTION UI (legacy — no longer called)
function params = parameterSelectionUI()
% Create a figure for parameter selection 
fig = figure('Name', 'Parameter Selection', 'NumberTitle', 'off', ...
    'Position', [300 50 520 1150], 'MenuBar', 'none', 'ToolBar', 'none');

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
params.refcycle = 1; 
params.skipAutoInverse = false; 
params.skipManualInverse = false; 
params.createSolidMesh = true;
params.enableOverlapFilter = false;
params.overlapRadius = 0.5;
params.usePoissonRecon = true;
params.poissonReconPath = '';
% Auto-detect PoissonRecon.exe in the same folder as this script
scriptDir = fileparts(mfilename('fullpath'));
autoPath = fullfile(scriptDir, 'PoissonRecon.exe');
if exist(autoPath, 'file')
    params.poissonReconPath = autoPath;
    fprintf('PoissonRecon.exe auto-detected at: %s\n', autoPath);
end
params.poissonDepth = 10;
params.poissonPointWeight = 0;
params.poissonNormalNeighbors = 20;
params.enablePoissonTrimmer = true;
params.poissonTrimValue = 7;

% UI Controls
y_start = 1100;
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
% Create Solid Mesh Checkbox
chk_solid = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ... 
    'String', 'Create Solid Block (Extrude)', 'Value', params.createSolidMesh, ...
    'Callback', @toggleSolidParams); 
 
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

y_start = y_start - spacing;
% Refinement Cycles
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Refinement cycles:', 'HorizontalAlignment', 'right');
edit_refcycle = uicontrol('Style', 'edit', 'Position', [220 y_start 150 h], 'String', num2str(params.refcycle));

y_start = y_start - spacing * 1.5;
% Skip Manual Inverse Rotation Checkbox
chk_skipManualInv = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Skip Manual Inverse Rotation', 'Value', params.skipManualInverse);

y_start = y_start - spacing;
% Skip Automatic Inverse Rotation Checkbox
chk_skipAutoInv = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Skip Automatic Inverse Rotation', 'Value', params.skipAutoInverse);

y_start = y_start - spacing;
% Translate to Origin after Alignment Checkbox
chk_transToOrigin = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Translate to Origin after Alignment', 'Value', false);

y_start = y_start - spacing;
% Skip Plane Cutting Checkbox
chk_skipCutting = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Skip Plane Cutting (Keep Alignment)', 'Value', false);

y_start = y_start - spacing;
% Enable Overlap Filter Checkbox
chk_overlap = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Enable Overlap Filter (Keep Highest)', 'Value', false, ...
    'Callback', @toggleOverlapParams);

y_start = y_start - spacing;
% Overlap Radius
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Overlap Radius:', 'HorizontalAlignment', 'right');
edit_overlapRad = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], 'String', '0.5', 'Enable', 'off');

y_start = y_start - spacing * 1.5;
% --- PoissonRecon Section ---
uicontrol('Style', 'text', 'Position', [50 y_start 400 h], ...
    'String', '--- PoissonRecon Settings ---', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

y_start = y_start - spacing;
% Enable PoissonRecon Checkbox
chk_poisson = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Use PoissonRecon (recommended)', 'Value', params.usePoissonRecon, ...
    'Callback', @togglePoissonParams);

y_start = y_start - spacing;
% PoissonRecon Executable Path
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'PoissonRecon.exe:', 'HorizontalAlignment', 'right');
edit_poissonPath = uicontrol('Style', 'edit', 'Position', [220 y_start 200 h], ...
    'String', params.poissonReconPath);
uicontrol('Style', 'pushbutton', 'Position', [425 y_start 40 h], 'String', '...', ...
    'Callback', @browsePoissonExe);

y_start = y_start - spacing;
% Poisson Depth
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Poisson Depth (8-12):', 'HorizontalAlignment', 'right');
edit_poissonDepth = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], ...
    'String', num2str(params.poissonDepth));

y_start = y_start - spacing;
% Poisson Point Weight
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Point Weight (0=interpolate):', 'HorizontalAlignment', 'right');
edit_poissonWeight = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], ...
    'String', num2str(params.poissonPointWeight));

y_start = y_start - spacing;
% Normal Estimation Neighbors
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Normal Neighbors:', 'HorizontalAlignment', 'right');
edit_poissonNeighbors = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], ...
    'String', num2str(params.poissonNormalNeighbors));

y_start = y_start - spacing;
% Enable SurfaceTrimmer Checkbox
chk_trimmer = uicontrol('Style', 'checkbox', 'Position', [220 y_start 250 h], ...
    'String', 'Run SurfaceTrimmer (remove artifacts)', 'Value', params.enablePoissonTrimmer, ...
    'Callback', @toggleTrimmerParams);

y_start = y_start - spacing;
% Trim Value
uicontrol('Style', 'text', 'Position', [50 y_start 150 h], 'String', 'Trim Value (5-10):', 'HorizontalAlignment', 'right');
edit_poissonTrim = uicontrol('Style', 'edit', 'Position', [220 y_start 100 h], ...
    'String', num2str(params.poissonTrimValue));
uicontrol('Style', 'text', 'Position', [325 y_start 140 h], ...
    'String', '(low=keep more, high=aggressive)', 'FontSize', 7, 'HorizontalAlignment', 'left');

y_start = y_start - spacing * 2;
% Start Button
uicontrol('Style', 'pushbutton', 'Position', [150 y_start 200 40], 'String', 'Start Processing', ...
    'FontSize', 12, 'Callback', @startProcessing);

% --- Callbacks ---

function toggleWeldingParams(src, ~)
    state = get(src, 'Value');
    if state
        set(edit_weldAngle,  'Enable', 'on');
        set(edit_peakThresh, 'Enable', 'on');
    else
        set(edit_weldAngle,  'Enable', 'off');
        set(edit_peakThresh, 'Enable', 'off');
    end
end

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

function toggleOverlapParams(src, ~)
    state = get(src, 'Value');
    if state
        set(edit_overlapRad, 'Enable', 'on');
    else
        set(edit_overlapRad, 'Enable', 'off');
    end
end

function toggleSolidParams(src, ~)
    state = get(src, 'Value');
    if state
        set(edit_zdeep, 'Enable', 'on');
    else
        set(edit_zdeep, 'Enable', 'off');
    end
end

function togglePoissonParams(src, ~)
    state = get(src, 'Value');
    en = 'off'; if state, en = 'on'; end
    set(edit_poissonPath,      'Enable', en);
    set(edit_poissonDepth,     'Enable', en);
    set(edit_poissonWeight,    'Enable', en);
    set(edit_poissonNeighbors, 'Enable', en);
    set(chk_trimmer,           'Enable', en);
    if state && get(chk_trimmer, 'Value')
        set(edit_poissonTrim, 'Enable', 'on');
    else
        set(edit_poissonTrim, 'Enable', 'off');
    end
end

function toggleTrimmerParams(src, ~)
    state = get(src, 'Value');
    if state
        set(edit_poissonTrim, 'Enable', 'on');
    else
        set(edit_poissonTrim, 'Enable', 'off');
    end
end

function browsePoissonExe(~, ~)
    [f, p] = uigetfile({'PoissonRecon.exe;*.exe', 'Executable (*.exe)'; '*.*', 'All Files'}, ...
        'Select PoissonRecon.exe');
    if f ~= 0
        set(edit_poissonPath, 'String', fullfile(p, f));
    end
end

function startProcessing(~, ~)
    params.gridStep             = str2double(get(edit_gridStep, 'String'));
    params.max_edge_length      = str2double(get(edit_edge, 'String'));
    params.z_deep               = str2double(get(edit_zdeep, 'String'));
    params.planeDistance        = str2double(get(edit_planeDist, 'String'));
    params.enableWelding        = get(chk_welding, 'Value');
    params.weldingAngle         = str2double(get(edit_weldAngle, 'String'));
    params.peakThreshold        = str2double(get(edit_peakThresh, 'String'));
    params.enableDownsampling   = get(chk_downsample, 'Value');
    params.enableInterpolation  = get(chk_interp, 'Value');
    interpOpts = {'linear', 'nearest', 'natural'};
    extrapOpts = {'linear', 'nearest', 'none'};
    params.interpolationMethod  = interpOpts{get(popup_interp, 'Value')};
    params.extrapolationMethod  = extrapOpts{get(popup_extrap, 'Value')};
    params.refcycle             = str2double(get(edit_refcycle, 'String'));
    params.skipManualInverse    = get(chk_skipManualInv, 'Value');
    params.skipAutoInverse      = get(chk_skipAutoInv, 'Value');
    params.translateToOrigin    = get(chk_transToOrigin, 'Value');
    params.skipPlaneCutting     = get(chk_skipCutting, 'Value');
    params.enableOverlapFilter  = get(chk_overlap, 'Value');
    params.overlapRadius        = str2double(get(edit_overlapRad, 'String'));
    params.createSolidMesh      = get(chk_solid, 'Value');
    params.usePoissonRecon      = get(chk_poisson, 'Value');
    params.poissonReconPath     = strtrim(get(edit_poissonPath, 'String'));
    params.poissonDepth         = str2double(get(edit_poissonDepth, 'String'));
    params.poissonPointWeight   = str2double(get(edit_poissonWeight, 'String'));
    params.poissonNormalNeighbors = str2double(get(edit_poissonNeighbors, 'String'));
    params.enablePoissonTrimmer  = get(chk_trimmer, 'Value');
    params.poissonTrimValue      = str2double(get(edit_poissonTrim, 'String'));
    uiresume(fig);
    close(fig);
end

% Wait for user input
addVerticalScroll(fig);
uiwait(fig);
end

%% SCROLLABLE FIGURE
function addVerticalScroll(fig)
% If the figure is taller than the screen, shrink it to fit and move its
% content into a panel that scrolls with a slider on the right side
% (and the mouse wheel, when the pointer is not over an axes).
sliderW = 18;
screenMargin = 120;   % room for taskbar, title bar and menu/toolbar

set(fig, 'Units', 'pixels');
figPos = get(fig, 'Position');
screen = get(0, 'ScreenSize');
maxH = screen(4) - screenMargin;
if figPos(4) <= maxH
    return;
end

contentW = figPos(3);
contentH = figPos(4);

% Move all layout children into a panel with the original figure size,
% so pixel and normalized positions stay exactly as designed
content = uipanel('Parent', fig, 'Units', 'pixels', 'BorderType', 'none', ...
    'Position', [0 0 contentW contentH]);
movable = {'uicontrol', 'axes', 'uipanel', 'uibuttongroup', 'uitable'};
kids = get(fig, 'Children');
for k = 1:numel(kids)
    if kids(k) ~= content && any(strcmp(get(kids(k), 'Type'), movable))
        set(kids(k), 'Parent', content);
    end
end

scrollBar = uicontrol('Parent', fig, 'Style', 'slider', 'Units', 'pixels', ...
    'Callback', @(~, ~) applyScroll());
addlistener(scrollBar, 'ContinuousValueChange', @(~, ~) applyScroll());

% Fit the window to the screen, keeping the top edge visible
set(fig, 'Position', [figPos(1), screenMargin / 2, contentW + sliderW, maxH]);

layout();
set(fig, 'SizeChangedFcn', @(~, ~) layout());
if isempty(get(fig, 'WindowScrollWheelFcn'))
    set(fig, 'WindowScrollWheelFcn', @onWheel);
end

    function layout()
        pos = get(fig, 'Position');
        viewH = pos(4);
        set(content, 'Position', [0, 0, max(contentW, pos(3) - sliderW), contentH]);
        set(scrollBar, 'Position', [pos(3) - sliderW, 0, sliderW, viewH]);
        scrollRange = contentH - viewH;
        if scrollRange > 0
            set(scrollBar, 'Enable', 'on', 'Min', 0, 'Max', scrollRange, ...
                'Value', scrollRange, ...   % start at the top
                'SliderStep', [min(1, 40 / scrollRange), min(1, viewH / scrollRange)]);
        else
            set(scrollBar, 'Enable', 'off', 'Min', 0, 'Max', 1, 'Value', 1);
        end
        applyScroll();
    end

    function applyScroll()
        pos = get(fig, 'Position');
        if strcmp(get(scrollBar, 'Enable'), 'on')
            offset = -get(scrollBar, 'Value');
        else
            offset = pos(4) - contentH;   % content fits: align to top
        end
        contentPos = get(content, 'Position');
        set(content, 'Position', [0, offset, contentPos(3), contentH]);
    end

    function onWheel(~, evt)
        if strcmp(get(scrollBar, 'Enable'), 'off')
            return;
        end
        % Leave the wheel to the axes (3D zoom) when the pointer is over one
        try
            if ~isempty(ancestor(hittest(fig), 'axes'))
                return;
            end
        catch
        end
        newVal = get(scrollBar, 'Value') - evt.VerticalScrollCount * 40;
        set(scrollBar, 'Value', min(max(newVal, get(scrollBar, 'Min')), get(scrollBar, 'Max')));
        applyScroll();
    end
end
