% ======
% ROB521_assignment1.m
% ======
%
% This assignment will introduce you to the idea of motion planning for  
% holonomic robots that can move in any direction and change direction of 
% motion instantaneously.  Although unrealistic, it can work quite well for
% complex large scale planning.  You will generate mazes to plan through 
% and employ the PRM algorithm presented in lecture as well as any 
% variations you can invent in the later sections.
% 
% There are three questions to complete (5 marks each):
%
%    Question 1: implement the PRM algorithm to construct a graph
%    connecting start to finish nodes.
%    Question 2: find the shortest path over the graph by implementing the
%    Dijkstra's or A* algorithm.
%    Question 3: identify sampling, connection or collision checking 
%    strategies that can reduce runtime for mazes.
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plots, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file.
%
% requires: basic Matlab, 
%
% S L Waslander, January 2022
%
clear; close all; clc;

% set random seed for repeatability if desired
rng(1);
%%
% ==========================
% Maze Generation
% ==========================
%
% The maze function returns a map object with all of the edges in the maze.
% Each row of the map structure draws a single line of the maze.  The
% function returns the lines with coordinates [x1 y1 x2 y2].
% Bottom left corner of maze is [0.5 0.5], 
% Top right corner is [col+0.5 row+0.5]
%

row = 5; % Maze rows
col = 7; % Maze columns
map = maze(row,col); % Creates the maze
start = [0.5, 1.0]; % Start at the bottom left
finish = [col+0.5, row]; % Finish at the top right

h = figure(1);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

% ======================================================
% Question 1: construct a PRM connecting start and finish
% ======================================================
%
% Using 500 samples, construct a PRM graph whose milestones stay at least 
% 0.1 units away from all walls, using the MinDist2Edges function provided for 
% collision detection.  Use a nearest neighbour connection strategy and the 
% CheckCollision function provided for collision checking, and find an 
% appropriate number of connections to ensure a connection from  start to 
% finish with high probability.


% variables to store PRM components
nS = 500;  % number of samples to try for milestone creation
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

disp("Time to create PRM graph")
tic;
% ------insert your PRM generation code here-------
max_dist = col^2 + row^2 + 10;
n_nearest = 8;

% generating milestones
points_x = rand([nS 1]) * (col - 0.1 * 2) + 0.5 + 0.1;
points_y = rand([nS 1]) * (row - 0.1 * 2) + 0.5 + 0.1;

temp_milestones = [points_x points_y];
n_points = length(temp_milestones);

for i=1:n_points
    if MinDist2Edges([temp_milestones(i, :)], map) >= 0.1
        milestones = [milestones ; temp_milestones(i, :)];
    end
end

n_points = length(milestones);

% connecting milestones
distances = zeros([n_points n_points]);

for i=1:n_points
    for j=1:n_points
        diff = milestones(i, :) - milestones(j, :);
        distances(i, j) = diff * diff';
    end
    distances(i, i) = max_dist;
end

for i=1:n_points
    for j=1:n_nearest
        [val, idx] = min(distances(i, :));
        distances(i, idx) = max_dist;
        distances(idx, i) = max_dist;
        temp_edge = [milestones(i, :) milestones(idx, :)];
        collide = 0;
        for k=1:length(map)
            collide = collide | EdgeCollision(temp_edge, map(k, :), 0.1);
        end
        if ~collide
            edges = [edges ; temp_edge];
        end
    end
end

% ------end of your PRM generation code -------
toc;

figure(1);
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
end
str = sprintf('Q1 - %d X %d Maze PRM', row, col);
title(str);
drawnow;

print -dpng assignment1_q1.png

%%
% =================================================================
% Question 2: Find the shortest path over the PRM graph
% =================================================================
%
% Using an optimal graph search method (Dijkstra's or A*) , find the 
% shortest path across the graph generated.  Please code your own 
% implementation instead of using any built in functions.

disp('Time to find shortest path');
tic;

% Variable to store shortest path
spath = []; % shortest path, stored as a milestone row index sequence


% ------insert your shortest path finding algorithm here-------

ms_index = linspace(1, n_points);
distances = zeros([n_points n_points]);
for i=1:length(edges)
    p1 = edges(i, 1:2);
    p2 = edges(i, 3:4);
    idx1 = find(milestones(:, 1) == p1(1) & milestones(:, 2) == p1(2));
    idx2 = find(milestones(:, 1) == p2(1) & milestones(:, 2) == p2(2));
    diff = p1 - p2;
    distances(idx1, idx2) = diff * diff';
    distances(idx2, idx1) = diff * diff';
end

sqr_diff = (milestones - finish).^2;
heuristic = sqrt(sqr_diff(:, 1) + sqr_diff(:, 2));

visited = zeros(n_points);
visited(1) = 1;

score = zeros(n_points) + max_dist * 999;
score(1) = 0;
prev_pts = zeros(n_points);
dead = [];
queue = [1];

while ~isempty(queue)
    cur = queue(1);
    queue(1) = [];
    if cur == 2
        break
    end
    next_pts = find(distances(cur, :) ~= 0);
    cur_score = score(cur);
    for pts=next_pts
        s = cur_score + heuristic(pts);
        if score(pts) > s
            score(pts) = s;
            prev_pts(pts) = cur;
            queue = [queue pts];
        end
    end
    queue = unique(queue);
end

cur = 2;
while cur ~= 0
    spath = [cur spath];
    cur = prev_pts(cur);
end


% ------end of shortest path finding algorithm------- 
toc;    

% plot the shortest path
figure(1);
for i=1:length(spath)-1
    plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
end
str = sprintf('Q2 - %d X %d Maze Shortest Path', row, col);
title(str);
drawnow;

print -dpng assingment1_q2.png

%%
% ================================================================
% Question 3: find a faster way
% ================================================================
%
% Modify your milestone generation, edge connection, collision detection 
% and/or shortest path methods to reduce runtime.  What is the largest maze 
% for which you can find a shortest path from start to goal in under 20 
% seconds on your computer? (Anything larger than 40x40 will suffice for 
% full marks)


row = 25;
col = 25;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;        
% ------insert your optimized algorithm here------









% ------end of your optimized algorithm-------
dt = toc;

figure(2); hold on;
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta')
end
if (~isempty(spath))
    for i=1:length(spath)-1
        plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
    end
end
str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
title(str);

print -dpng assignment1_q3.png

