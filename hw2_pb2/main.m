clear ; close all; clc;

% Scaling down by a factor of 2, otherwise too slow.
left_img = imresize(imread('image/left.png'), 0.5);
right_img = imresize(imread('image/right.png'), 0.5);
K = load('K.txt');
K(1:2, :) = K(1:2, :) / 2;

% Given by the KITTI dataset:
baseline = 0.54;

% Carefully tuned! Do not modify!
patch_radius = 5;
min_disp = 5;
max_disp = 50;
xlims = [7 20];
ylims = [-6 10];
zlims = [-5 5];

%% Parts 1

tic;
disp_img = getDisparity(...
    left_img, right_img, patch_radius, min_disp, max_disp);
toc
figure(1);
imagesc(disp_img);
axis equal;
axis off;


%% Part 3: Create point cloud for first pair

[p_C_points, intensities] = disparityToPointCloud(...
    disp_img, K, baseline, left_img);
% From camera frame to world frame:
p_F_points = [0 -1 0; 0 0 -1; 1 0 0]^-1 * p_C_points(:, 1:10:end);

figure(3);
scatter3(p_F_points(1, :), p_F_points(2, :), p_F_points(3, :), ...
    20 * ones(1, length(p_F_points)), ...
    repmat(single(intensities(1:10:end))'/255, [1 3]), 'filled');
axis equal;
axis([0 30 ylims zlims]);
axis vis3d;
grid off;
xlabel('X');
ylabel('Y');
zlabel('Z');
