% Input --> Image : 3 channel RBG image matrix
%{
output
x : n x 1 vector, 

각 step : 자율주행시스템 개론 강의자료를 참고함.
%}
function [ x, y, scores, Ix, Iy] = harris( image )
% (step 1) compute derivatices in x and y directions (Ix,Iy) e.g. with Sobel filter

sobel_x = [ 1 0 -1; 2 0 -2; 1 0 -1];
sobel_y = [ 1 2 1; 0 0 0 ; -1 -2 -1];

filtered_x = imfilter(image, sobel_x);
filtered_y = imfilter(image, sobel_y);

Ix = filtered_x;
Iy = filtered_y;

%{
(step 2) compute Ix^2, Iy^2, IxIy
(step 3) convolve Ix^2, Iy^2, IxIy with a box filter to get sigma(Ix^2),
sigma(Iy^2), sigma(IxIy), which are the entries of the matrix M
(optionally use a Gaussian filter instead of a box filter to avoid
aliasing and give more "weight" to the central pixels)
(step 4) compute harris corner measure R (according to Shi-Tomasi or Harris)
%}
f = fspecial("gaussian");
Ix2 = imfilter(Ix.^2, f);
Iy2 = imfilter(Iy.^2, f);
IxIy = imfilter(Ix.*Iy, f);

k = 0.04;
num_rows = size(image, 1);
num_cols = size(image, 2);

H = zeros(num_rows, num_cols);

% get matrix M for each pixel
for y = 6:num_rows-6         % avoid edges
    for x = 6:num_cols-6     % avoid edges  
        % calculate means -->  mean is sum/num pixels
        % Ix2 mean
        Ix2_matrix = Ix2(y-2:y+2,x-2:x+2);
        Ix2_mean = sum(Ix2_matrix(:));
        
        % Iy2 mean
        Iy2_matrix = Iy2(y-2:y+2,x-2:x+2);
        Iy2_mean = sum(Iy2_matrix(:));
        
        % IxIy mean
        IxIy_matrix = IxIy(y-2:y+2,x-2:x+2);
        IxIy_mean = sum(IxIy_matrix(:));
        
        % compute R, using te matrix we just created
        Matrix = [Ix2_mean, IxIy_mean; 
                  IxIy_mean, Iy2_mean];
        R1 = det(Matrix) - (k * trace(Matrix)^2);
        
        % store the R values in our Harris Matrix
        H(y,x) = R1;
       
    end
end

% (step 5) Find points with large corner response (R > threshold)
% (step 6) Take the points of local maxima of R
avg_r = mean(mean(H));
threshold = abs(5 * avg_r);    % abs : absolute value
[row, col] = find(H > threshold);
scores = [];

% get all the values
for index = 1:size(row,1)
    %see what the values are
    r = row(index);
    c = col(index);
    scores = cat(2, scores,H(r,c));
end

y = row;
x = col;

end


