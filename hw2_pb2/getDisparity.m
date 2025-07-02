function disp_img = getDisparity(...
    left_img, right_img, patch_radius, min_disp, max_disp)
% left_img and right_img are both H x W and you should return a H x W
% matrix containing the disparity d for each pixel of left_img. Set
% disp_img to 0 for pixels where the SSD and/or d is not defined, and for d
% estimates rejected in Part 2. patch_radius specifies the SSD patch and
% each valid d should satisfy min_disp <= d <= max_disp.

r = patch_radius;
patch_size = 2 * patch_radius + 1;

disp_img = zeros(size(left_img));

rows = size(left_img, 1);
cols = size(left_img, 2);

for row = (1 + patch_radius):(rows-patch_radius)
    for col = (1 + max_disp + patch_radius):(cols - patch_radius)

        left_patch = single(left_img(%your code here ,%your code here));
        right_strip = single(right_img(%your code here , %your code here));
        
        % Transforming the patches into vectors so we can run them through
        % pdist2.
        lpvec = single(left_patch(:));
        rsvecs = single(zeros(patch_size^2, max_disp - min_disp + 1));
        for i = 1:patch_size
            rsvecs(((i-1)*patch_size+1):(i*patch_size), :) = ...
                right_strip(:, %your code here);
        end
        
        ssds = pdist2(lpvec', rsvecs', 'squaredeuclidean');
        
        % The way the patches are set up, the argmin of ssds will not
        % directly be the disparity, but rather (max_disparity -
        % disparity). We call this "neg_disp".
        [min_ssd, neg_disp] = %your code here
        
        if (nnz(ssds <= 1.5 * min_ssd) < 3 && neg_disp ~= 1 && ...
                neg_disp ~= length(ssds))

        disp_img(row, col) = max_disp - neg_disp;

        else

        disp_img(row, col) =0;
        
        end
    end
end

end

