function compare_segmentations(basedir, seg_dir1, seg_dir2)

close all; clc;

if nargin < 1
    basedir = '../segmentation_output';
end

if nargin < 3
    % Floodfill baseline
    seg_dir1 = [basedir '/floodfill/trk_eval/'];    

    % Euclidean clustering baseline
    %seg_dir1 = [basedir '/ec1/trk_eval/'];    
    %seg_dir1 = [basedir '/ec2/trk_eval/'];    
    %seg_dir1 = [basedir '/ec3/trk_eval/'];    
    %seg_dir1 = [basedir '/ec4/trk_eval/'];    
    %seg_dir1 = [basedir '/ec5/trk_eval/'];    
    
    % Wang, et al. baseline
    %seg_dir1 = [basedir '/newman1/trk_eval/'];    
    %seg_dir1 = [basedir '/newman2/trk_eval/'];    
    %seg_dir1 = [basedir '/newman3/trk_eval/'];    
    %seg_dir1 = [basedir '/newman4/trk_eval/'];    

    % Ablation - different versions of our method
    %seg_dir2 = [basedir '/floodfill+temporal/trk_eval/'];    
    %seg_dir2 = [basedir '/floodfill+semantic/trk_eval/'];    
    
    % Our full method, optimizing for maximizing intersection-over-union
    %seg_dir2 = [basedir '/floodfill+semantic+temporal+iou/trk_eval/'];    

    % Our full method, optimizing for minimizing total under / oversegmentation errors
    seg_dir2 = [basedir '/floodfill+semantic+temporal/trk_eval/'];    
end

basedir = '/home/davheld/data/segmentation/runs';

% Analyze separately all segments within this distance from the Velodyne (meters).
distance_thresh = 15;

[total_errors1, total_errors_distance1, num_id_switches1, num_id_switches_nearby1] = segmentation_score(basedir, seg_dir1, distance_thresh);
fprintf('\n');
[total_errors2, total_errors_distance2, num_id_switches2, num_id_switches_nearby2] = segmentation_score(basedir, seg_dir2, distance_thresh);

fprintf('\n');
overall_improvement = (total_errors1 - total_errors2)/ total_errors1;
improvement_within_distance = (total_errors_distance1 - total_errors_distance2) / total_errors_distance1;
display(['Relative segmentation improvements: Overall: ' num2str(overall_improvement) ', Within ' num2str(distance_thresh) 'm: ' num2str(improvement_within_distance)])

id_switch_improvements = (num_id_switches1 - num_id_switches2) / num_id_switches1;
id_switch_improvements_nearby = (num_id_switches_nearby1 - num_id_switches_nearby2) / num_id_switches_nearby1;
display(['Relative ID switch improvements: Overall: ' num2str(id_switch_improvements) ', Within ' num2str(distance_thresh) 'm: ' num2str(id_switch_improvements_nearby)])

end