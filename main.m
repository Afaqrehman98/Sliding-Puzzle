function sliding_puzzle_gui
    % Initialize the figure
    fig = uifigure('Position', [100, 100, 300, 400], 'Name', 'Sliding Puzzle');
    
    % Create a grid layout for the puzzle
    gl = uigridlayout(fig, [3, 2]);
    gl.RowHeight = {'1x', '1x', '1x'};
    gl.ColumnWidth = {'1x', '1x'};
    
    % Initialize puzzle state
    start_state_2x2 = [0, 3; 2, 1];
    goal_state_2x2 = [1, 2; 3, 0];
    
    % Create UI components for the puzzle tiles
    tiles = gobjects(2, 2);
    for i = 1:2
        for j = 1:2
            tiles(i, j) = uibutton(gl, 'Text', num2str(start_state_2x2(i, j)), 'ButtonPushedFcn', @(btn, event) tile_callback(btn, i, j));
            tiles(i, j).FontSize = 20;
            if start_state_2x2(i, j) == 0
                tiles(i, j).Text = '';
                tiles(i, j).BackgroundColor = [0.8, 0.8, 0.8];
            end
        end
    end
    
    % Create buttons for A* and brute force search
    a_star_button = uibutton(gl, 'Text', 'A* Search', 'ButtonPushedFcn', @(btn, event) solve_puzzle('a_star', start_state_2x2, goal_state_2x2, tiles));
    brute_force_button = uibutton(gl, 'Text', 'Brute Force', 'ButtonPushedFcn', @(btn, event) solve_puzzle('brute_force', start_state_2x2, goal_state_2x2, tiles));
    
    function tile_callback(btn, row, col)
        % Handle tile click event to update puzzle state
        empty_pos = find(start_state_2x2 == 0);
        [empty_row, empty_col] = ind2sub(size(start_state_2x2), empty_pos);
        if abs(row - empty_row) + abs(col - empty_col) == 1
            start_state_2x2(empty_row, empty_col) = start_state_2x2(row, col);
            start_state_2x2(row, col) = 0;
            update_tiles();
        end
    end

    function update_tiles()
        % Update the UI buttons based on the current puzzle state
        for i = 1:2
            for j = 1:2
                tiles(i, j).Text = num2str(start_state_2x2(i, j));
                if start_state_2x2(i, j) == 0
                    tiles(i, j).Text = '';
                    tiles(i, j).BackgroundColor = [0.8, 0.8, 0.8];
                else
                    tiles(i, j).BackgroundColor = [1, 1, 1];
                end
            end
        end
    end

    function solve_puzzle(method, start_state, goal_state, tiles)
        % Solve the puzzle using the specified method and display the solution
        if strcmp(method, 'a_star')
            solution = a_star_search(start_state, goal_state);
        else
            solution = brute_force_search(start_state, goal_state);
        end
        
        if isempty(solution)
            disp('No solution found!');
            return;
        end
        
        % Animate the solution
        for step = 1:length(solution)
            start_state = solution{step};
            for i = 1:2
                for j = 1:2
                    tiles(i, j).Text = num2str(start_state(i, j));
                    if start_state(i, j) == 0
                        tiles(i, j).Text = '';
                        tiles(i, j).BackgroundColor = [0.8, 0.8, 0.8];
                    else
                        tiles(i, j).BackgroundColor = [1, 1, 1];
                    end
                end
            end
            pause(0.5);
        end
    end

    function solution = a_star_search(start_state, goal_state)
        % Priority queue using a cell array
        pq = {start_state, 0};
        came_from = containers.Map();
        g_score = containers.Map();
        f_score = containers.Map();
        
        start_key = mat2str(start_state);
        goal_key = mat2str(goal_state);
        
        g_score(start_key) = 0;
        f_score(start_key) = manhattan_distance(start_state, goal_state);
        came_from(start_key) = '';
        
        while ~isempty(pq)
            % Extract the node with the lowest f_score
            [~, idx] = min(cell2mat(pq(:, 2)));
            current = pq{idx, 1};
            pq(idx, :) = [];
            current_key = mat2str(current);
            
            if isequal(current, goal_state)
                solution = reconstruct_path(came_from, current_key);
                return;
            end
            
            neighbors = get_neighbors(current);
            for i = 1:length(neighbors)
                neighbor = neighbors{i};
                neighbor_key = mat2str(neighbor);
                tentative_g_score = g_score(current_key) + 1;
                
                if ~isKey(g_score, neighbor_key) || tentative_g_score < g_score(neighbor_key)
                    came_from(neighbor_key) = current_key;
                    g_score(neighbor_key) = tentative_g_score;
                    f_score(neighbor_key) = tentative_g_score + manhattan_distance(neighbor, goal_state);
                    if ~any(cellfun(@(x) isequal(x, neighbor), pq(:, 1)))
                        pq{end + 1, 1} = neighbor;
                        pq{end, 2} = f_score(neighbor_key);
                    end
                end
            end
        end
        solution = [];
    end

    function solution = brute_force_search(start_state, goal_state)
        % Queue using a cell array
        q = {start_state};
        came_from = containers.Map();
        
        start_key = mat2str(start_state);
        goal_key = mat2str(goal_state);
        came_from(start_key) = '';
        
        visited = containers.Map();
        visited(start_key) = true;
        
        while ~isempty(q)
            current = q{1};
            q(1) = [];
            current_key = mat2str(current);
            
            if isequal(current, goal_state)
                solution = reconstruct_path(came_from, current_key);
                return;
            end
            
            neighbors = get_neighbors(current);
            for i = 1:length(neighbors)
                neighbor = neighbors{i};
                neighbor_key = mat2str(neighbor);
                
                if ~isKey(visited, neighbor_key)
                    visited(neighbor_key) = true;
                    came_from(neighbor_key) = current_key;
                    q{end + 1} = neighbor;
                end
            end
        end
        solution = [];
    end

    function path = reconstruct_path(came_from, current_key)
        path = {str2num(current_key)}; %#ok<ST2NM>
        while ~isempty(came_from(current_key))
            current_key = came_from(current_key);
            path{end + 1} = str2num(current_key); %#ok<ST2NM>
        end
        path = flip(path);
    end

    function neighbors = get_neighbors(state)
        % Find the empty space (0)
        [x, y] = find(state == 0);
        neighbors = {};
        directions = [0, 1; 0, -1; 1, 0; -1, 0];
        
        for i = 1:size(directions, 1)
            nx = x + directions(i, 1);
            ny = y + directions(i, 2);
            if nx >= 1 && nx <= size(state, 1) && ny >= 1 && ny <= size(state, 2)
                new_state = state;
                new_state(x, y) = new_state(nx, ny);
                new_state(nx, ny) = 0;
                neighbors{end + 1} = new_state;
            end
        end
    end

    function distance = manhattan_distance(state, goal_state)
        distance = 0;
        for x = 1:size(state, 1)
            for y = 1:size(state, 2)
                value = state(x, y);
                if value ~= 0
                    [target_x, target_y] = find(goal_state == value);
                    distance = distance + abs(x - target_x) + abs(y - target_y);
                end
            end
        end
    end
end