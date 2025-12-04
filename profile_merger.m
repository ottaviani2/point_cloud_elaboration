function profile_merger
    % PROFILE_MERGER Interactive tool to merge two topographic profiles.
    %   This script opens a GUI that allows the user to load two profiles
    %   (X, Y data), visualizes them, and provides a slider to interactively
    %   choose a cut-point. The resulting profile consists of Profile 1
    %   up to the cut-point, and Profile 2 thereafter. The result can be
    %   saved to a text file.
    %
    %   Usage:
    %       1. Run 'profile_merger' in the MATLAB command window.
    %       2. Click 'Load Profile 1' and select a text file (columns: X Y).
    %       3. Click 'Load Profile 2' and select the second text file.
    %       4. Use the slider to move the vertical cut line.
    %       5. The thick black line shows the combined profile.
    %       6. Click 'Save Result' to save the combined profile to a file.

    % Create the main figure
    f = figure('Name', 'Topographic Profile Merger', ...
               'NumberTitle', 'off', ...
               'Position', [100, 100, 900, 600], ...
               'MenuBar', 'none', ...
               'ToolBar', 'figure');

    % Data storage structure
    data = struct();
    data.x1 = []; data.y1 = [];
    data.x2 = []; data.y2 = [];
    data.x_combined = [];
    data.y_combined = [];
    

    % UI Components: Axes
    ax = axes('Parent', f, 'Position', [0.1, 0.3, 0.85, 0.65]);
    xlabel(ax, 'Distance (X)');
    ylabel(ax, 'Elevation (Y)');
    title(ax, 'Profile Comparison and Merge');
    grid(ax, 'on');
    hold(ax, 'on');

    % UI Components: Controls
    panel = uipanel('Parent', f, 'Position', [0.05, 0.05, 0.9, 0.15], 'Title', 'Controls');
    
    btnLoad1 = uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Load Profile 1', ...
        'Units', 'normalized', 'Position', [0.02, 0.55, 0.15, 0.35], 'Callback', @loadProfile1);
        
    btnLoad2 = uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Load Profile 2', ...
        'Units', 'normalized', 'Position', [0.18, 0.55, 0.15, 0.35], 'Callback', @loadProfile2);
        
    btnSave = uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Save Result', ...
        'Units', 'normalized', 'Position', [0.83, 0.55, 0.15, 0.35], 'Callback', @saveResult);

    % Slider
    % Slider with continuous update
    % Slider
    sld = uicontrol('Parent', panel, 'Style', 'slider', ...
        'Units', 'normalized', 'Position', [0.35, 0.55, 0.45, 0.35], ...
        'Enable', 'off', 'Callback', @updatePlot);
    
    % For continuous updates while dragging
    try
        % Try modern approach first (R2014b+)
        set(sld, 'ContinuousValueChange', @updatePlot);
    catch
        % Fallback to listener for older versions
        addlistener(sld, 'Value', 'PostSet', @(src, event) updatePlot(sld, []));
    end
    
    % Label for slider
    lbl = uicontrol('Parent', panel, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.35, 0.15, 0.45, 0.3], ...
        'String', 'Load two profiles to begin...', 'HorizontalAlignment', 'center');

    % Plot handles (initialized as empty)
    h1 = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1, 'DisplayName', 'Profile 1 (Keep Left)');
    h2 = plot(ax, NaN, NaN, 'r--', 'LineWidth', 1, 'DisplayName', 'Profile 2 (Keep Right)');
    h_combined = plot(ax, NaN, NaN, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Combined Result');
    
    % Vertical line for cut point
    % Using plot instead of xline for compatibility and easy handle management
    h_line = plot(ax, [NaN NaN], [NaN NaN], 'g-', 'LineWidth', 2, 'DisplayName', 'Cut Point');
    
    legend(ax, 'show', 'Location', 'best');

    % --- Callbacks ---

    function loadProfile1(~, ~)
        [file, path] = uigetfile({'*.txt;*.dat;*.csv', 'Data Files'; '*.*', 'All Files'}, 'Select Profile 1');
        if isequal(file, 0), return; end
        
        try
            full_path = fullfile(path, file);
            [x, y] = load_and_select_columns(full_path);
            
            if isempty(x) || isempty(y), return; end % Cancelled or error
            
            data.x1 = x;
            data.y1 = y;
            
            % Sort by X just in case
            [data.x1, sortIdx] = sort(data.x1);
            data.y1 = data.y1(sortIdx);
            
            updateData();
        catch ME
            errordlg(['Error loading file: ' ME.message], 'Load Error');
        end
    end

    function loadProfile2(~, ~)
        [file, path] = uigetfile({'*.txt;*.dat;*.csv', 'Data Files'; '*.*', 'All Files'}, 'Select Profile 2');
        if isequal(file, 0), return; end
        
        try
            full_path = fullfile(path, file);
            [x, y] = load_and_select_columns(full_path);
            
            if isempty(x) || isempty(y), return; end % Cancelled or error
            
            data.x2 = x;
            data.y2 = y;
            
            % Sort by X
            [data.x2, sortIdx] = sort(data.x2);
            data.y2 = data.y2(sortIdx);
            
            updateData();
        catch ME
            errordlg(['Error loading file: ' ME.message], 'Load Error');
        end
    end

    function updateData()
        % Calculate offset only once when BOTH profiles are loaded
        if ~isempty(data.x1) && ~isempty(data.x2)
            % Find the minimum X value across both profiles
            min_x_global = min([min(data.x1), min(data.x2)]);
            
            % Shift both profiles so they start from 0
            % Do this only if not already shifted
            if min(data.x1) ~= 0 || min(data.x2) ~= 0
                data.x1 = data.x1 - min_x_global;
                data.x2 = data.x2 - min_x_global;
            end
            
            % Update individual plots
            set(h1, 'XData', data.x1, 'YData', data.y1);
            set(h2, 'XData', data.x2, 'YData', data.y2);
            
            % Calculate global Min/Max for slider
            all_x = [data.x1; data.x2];
            min_x = min(all_x);
            max_x = max(all_x);
            
            if max_x - min_x < 1
                max_x = min_x + 100;  % Ensure minimum range
            end
            
            % Set slider properties
            set(sld, 'Enable', 'on', ...
                'Min', min_x, ...
                'Max', max_x, ...
                'Value', (min_x + max_x) / 2, ...
                'SliderStep', [0.01, 0.1]); 
            
            updatePlot();
            
        elseif ~isempty(data.x1)
            % Only profile 1 loaded
            set(h1, 'XData', data.x1, 'YData', data.y1);
            
        elseif ~isempty(data.x2)
            % Only profile 2 loaded
            set(h2, 'XData', data.x2, 'YData', data.y2);
        end
    
        % Adjust axes
        if ~isempty(data.x1) || ~isempty(data.x2)
            axis(ax, 'tight');
            xl = xlim(ax); yl = ylim(ax);
            dx = (xl(2)-xl(1))*0.05; dy = (yl(2)-yl(1))*0.05;
            if dx==0, dx=1; end
            if dy==0, dy=1; end
            axis(ax, [xl(1)-dx xl(2)+dx yl(1)-dy yl(2)+dy]);
        end
    end

    function updatePlot(~, ~)
        if isempty(data.x1) || isempty(data.x2), return; end
        
        cut_val = get(sld, 'Value');
        
        % Ensure cut_val is within bounds of current min/max slider settings
        cut_val = max(get(sld, 'Min'), min(cut_val, get(sld, 'Max')));
        
        % Logic:
        % Result = Points from Profile 1 where X <= cut_val
        %          Points from Profile 2 where X > cut_val
        % This approach preserves original data points without interpolation.
        
        idx1 = data.x1 <= cut_val;
        idx2 = data.x2 > cut_val;
        
        x_comb = [data.x1(idx1); data.x2(idx2)];
        y_comb = [data.y1(idx1); data.y2(idx2)];
        
        % Store combined data
        data.x_combined = x_comb;
        data.y_combined = y_comb;
        
        % Update graphics
        set(h_combined, 'XData', data.x_combined, 'YData', data.y_combined);
        
        yl = ylim(ax);
        set(h_line, 'XData', [cut_val cut_val], 'YData', yl);
        
        set(lbl, 'String', sprintf('Cut Position X: %.2f', cut_val));
    end

    function saveResult(~, ~)
        if isempty(data.y_combined)
            msgbox('No combined profile to save. Please load two profiles first.', 'Error', 'error');
            return;
        end
        
        [file, path] = uiputfile('*.txt', 'Save Profile As');
        if isequal(file, 0), return; end
        
        % Remove NaNs if any (optional, but usually good for output)
        valid_idx = ~isnan(data.y_combined);
        out_x = data.x_combined(valid_idx)+min_x_global;
        out_y = data.y_combined(valid_idx);
        
        output_matrix = [out_x, out_y];
        
        try
            writematrix_compat(output_matrix, fullfile(path, file));
            msgbox('File Saved Successfully', 'Success');
        catch ME
            errordlg(['Error saving file: ' ME.message], 'Save Error');
        end
    end

    % --- Helper Functions ---
    
    function [x, y] = load_and_select_columns(filename)
        x = []; y = [];
        
        % Try to read data and detect headers
        % importdata is generally good at handling headers and data
        try
            imported = importdata(filename);
        catch
             error('Could not read file.');
        end
        
        raw_data = [];
        headers = {};
        
        if isstruct(imported)
            if isfield(imported, 'data')
                raw_data = imported.data;
            end
            if isfield(imported, 'colheaders')
                headers = imported.colheaders;
            end
        elseif isnumeric(imported)
            raw_data = imported;
        end
        
        % Fallback if importdata failed to get numeric data (e.g. complex format)
        if isempty(raw_data)
             raw_data = read_data_compat(filename);
        end
        
        [rows, cols] = size(raw_data);
        if rows == 0
            error('File appears to be empty.');
        end
        
        if cols == 1
            % Single column: Assume it's Y, generate X
            y = raw_data(:, 1);
            x = (1:rows)';
            return;
        end
        
        % Generate headers if missing or count mismatch
        if isempty(headers) || length(headers) ~= cols
            headers = cell(1, cols);
            for i = 1:cols
                headers{i} = sprintf('Column %d', i);
            end
        end
        
        % Open Dialog for Column Selection
        [x_idx, y_idx] = select_columns_dialog(headers);
        
        if isempty(x_idx) || isempty(y_idx)
            return; % User cancelled
        end
        
        x = raw_data(:, x_idx);
        y = raw_data(:, y_idx);
    end

    function [x_idx, y_idx] = select_columns_dialog(headers)
        x_idx = []; y_idx = [];
        
        d_width = 300;
        d_height = 200;
        d = dialog('Position', [300 300 d_width d_height], 'Name', 'Select Columns');
        
        uicontrol('Parent', d, 'Style', 'text', 'Position', [20 150 260 20], ...
            'String', 'Select X (Distance) Column:', 'HorizontalAlignment', 'left');
            
        popupX = uicontrol('Parent', d, 'Style', 'popup', 'Position', [20 130 260 20], ...
            'String', headers, 'Value', 1);
            
        uicontrol('Parent', d, 'Style', 'text', 'Position', [20 90 260 20], ...
            'String', 'Select Y (Elevation) Column:', 'HorizontalAlignment', 'left');
            
        popupY = uicontrol('Parent', d, 'Style', 'popup', 'Position', [20 70 260 20], ...
            'String', headers, 'Value', min(2, length(headers)));
            
        uicontrol('Parent', d, 'Style', 'pushbutton', 'Position', [100 20 100 30], ...
            'String', 'OK', 'Callback', @close_callback);
            
        uiwait(d);
        
        function close_callback(~, ~)
            x_idx = get(popupX, 'Value');
            y_idx = get(popupY, 'Value');
            delete(d);
        end
    end

    function data = read_data_compat(filename)
        % Wrapper to support older MATLAB versions
        if exist('readmatrix', 'file')
            data = readmatrix(filename);
        else
            data = dlmread(filename);
        end
    end

    function writematrix_compat(M, filename)
        % Wrapper to support older MATLAB versions
        if exist('writematrix', 'file')
            writematrix(M, filename, 'Delimiter', '\t');
        else
            dlmwrite(filename, M, 'delimiter', '\t', 'precision', 6);
        end
    end
end
