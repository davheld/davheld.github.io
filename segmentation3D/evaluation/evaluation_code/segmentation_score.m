function [total_errors, total_errors_distance, num_id_switches, num_id_switches_nearby] = segmentation_score(basedir, seg_dir, distance_thresh)

close all; 

if nargin < 1
    basedir = '../segmentation_output';
end

if nargin < 2
    % Floodfill baseline
    %seg_dir = [basedir '/floodfill/trk_eval/'];    

    % Euclidean clustering baseline
    %seg_dir = [basedir '/ec1/trk_eval/'];    
    %seg_dir = [basedir '/ec2/trk_eval/'];    
    %seg_dir = [basedir '/ec3/trk_eval/'];  
    %seg_dir = [basedir '/ec4/trk_eval/'];  
    %seg_dir = [basedir '/ec5/trk_eval/'];
    
    % Wang, et al. baseline
    %seg_dir = [basedir '/newman1/trk_eval/'];    
    %seg_dir = [basedir '/newman2/trk_eval/'];    
    %seg_dir = [basedir '/newman3/trk_eval/'];    
    %seg_dir = [basedir '/newman4/trk_eval/'];    

    % Ablation - different versions of our method
    %seg_dir = [basedir '/floodfill+temporal/trk_eval/'];    
    %seg_dir = [basedir '/floodfill+semantic/trk_eval/'];  
    
    % Our full method, optimizing for maximizing intersection-over-union
    %seg_dir1 = [basedir '/floodfill+semantic+temporal+iou/trk_eval/'];    
   
    % Our full method, optimizing for minimizing total under / oversegmentation errors
    seg_dir = [basedir '/floodfill+semantic+temporal/trk_eval/'];    
end

if nargin < 3
    % Analyze separately all segments within this distance from the Velodyne (meters).
    distance_thresh = 15;
end

fprintf('Processing segmentation dir: %s\n', seg_dir);

% If true: only analyze segments that have a correct classification (useful
% for debugging)
filter_incorrect_label = false;

% Only count undersegmentations based on this threshold, to allow for annotation errors.
% Note that our paper incorrectly listed the underseg_thresh as 0.5,
% although our results were reported with an underseg_thresh of 2/3.
underseg_thresh = 2/3;

% Read the files and compute the segmentation info.
[types, pos_points, blob_points, other_pos_points, distance, is_id_switch, class_idx, has_overlap] = ...
    readSegmentationFiles(seg_dir);

% Compute the valid indices.
valid_indices = getValidIndices(pos_points, has_overlap, types, class_idx, filter_incorrect_label);

% Compute the oversegmentation and undersegmentation errors.
[total_errors, total_errors_distance] = computeOverUnderSegs(pos_points, blob_points, other_pos_points, distance, valid_indices, underseg_thresh, distance_thresh);

% Compute the intersection over union segmentation scores.
computeIntersectionOverUnion(pos_points, blob_points, other_pos_points, distance, distance_thresh, valid_indices);

% Compute the number of ID switches.
[num_id_switches, num_id_switches_nearby] = computeIDswitches(valid_indices, distance, distance_thresh, is_id_switch);

end

function [num_id_switches, num_id_switches_nearby] = computeIDswitches(valid_indices, distance, distance_thresh, is_id_switch)
% Get nearby indices.
nearby_indices = distance < distance_thresh; 

% Count the number of id switches.
is_id_switch_overall = is_id_switch(valid_indices);
is_id_switch_nearby = is_id_switch(valid_indices & nearby_indices);

% Print the number of ID switches.
fprintf('ID switches: %s, percent: %s\n', num2str(sum(is_id_switch_overall)), num2str(mean(is_id_switch_overall)));
fprintf('ID switches (%s meters): %s, percent: %s\n', num2str(distance_thresh), num2str(sum(is_id_switch_nearby)), num2str(mean(is_id_switch_nearby)));

num_id_switches = sum(is_id_switch_overall);
num_id_switches_nearby = sum(is_id_switch_nearby);

end

function [total_errors, total_errors_distance] = computeOverUnderSegs(pos_points, blob_points, other_pos_points, distance, valid_indices, underseg_thresh, distance_thresh)
% Find over and under segmentations - see evaluation doc for explanation.
is_under_segmentation = (pos_points ./ blob_points < underseg_thresh) & valid_indices;
is_over_segmentation = (pos_points ./ (pos_points + other_pos_points) < 1) & valid_indices; 

% Find under and over segmentations within the threshold distance.
is_near_underseg = is_under_segmentation & distance < distance_thresh; 
is_near_overseg = is_over_segmentation & distance < distance_thresh;

% Compute the percentage of segmentation errors.
num_under_seg = sum(is_under_segmentation) / sum(valid_indices) * 100;
num_under_seg_distance = sum(is_near_underseg) / sum(valid_indices & distance < distance_thresh) * 100;
num_over_seg = sum(is_over_segmentation) / sum(valid_indices) * 100;
num_over_seg_distance = sum(is_near_overseg) / sum(valid_indices & distance < distance_thresh) * 100;

total_errors = num_under_seg + num_over_seg;
total_errors_distance = num_under_seg_distance + num_over_seg_distance;

% Print the number of over and undersegmentation errors.
display(['Undersegmentations: ' num2str(num_under_seg) ', Oversegmentations: ' num2str(num_over_seg), ', Total: ' num2str(total_errors)]);
display(['Within ' num2str(distance_thresh) 'm: Undersegmentations: ' num2str(num_under_seg_distance) ', Oversegmentations: ' num2str(num_over_seg_distance), ', Total: ' num2str(num_under_seg_distance + num_over_seg_distance)]);

end

% Remove '.' and '..'
function listing = listingPrune(listing)

for k = length(listing):-1:1
% remove folders starting with .
    fname = listing(k).name;
    if fname(1) == '.'
        listing(k) = [ ];
    end
end
end

% Read the files and compute the segmentation info.
function [types, pos_points, blob_points, other_pos_points, distance, is_id_switch, class_idx, has_overlap] = readSegmentationFiles(seg_dir)

  % Find all evaluation files.
  files = dir(seg_dir);

  % Remove '.' and '..'
  files = listingPrune(files);
  
  if (length(files) == 0)
      fprintf('Error - cannot find any files in directory: %s\n', seg_dir);
      return;
  end

  % Initialize variables to hold the info
  types = [];
  pos_points = [];
  blob_points = [];
  other_pos_points = [];
  distance = [];
  is_id_switch = [];
  class_idx = [];  
  has_overlap = [];  

% Iterate over all files.
for j = 1:length(files)
    
  %Open the file.
  filename = strcat(seg_dir, files(j).name);
  fid = fopen(filename);
  
  % Read header line.
  fgets(fid);
  
  % Read the data.
  C = textscan(fid,'%f %s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
  
  % Close the file.
  fclose(fid);
  
  % Process the filename to get the track number.
  tracknum = files(j).name; 
  tracknum = str2double(tracknum(1:4));

  % Ignore tracks 1 and 13 which are used for training.
  if ((tracknum == 1)  || (tracknum == 13))      
      continue
  end  
  
  % See the evaluation document for an explanation of these columns.
   types = [types; C{2}];
   pos_points = [pos_points; C{4}]; 
   blob_points = [blob_points; C{5}];
   other_pos_points = [other_pos_points; C{7}];
   distance = [distance; C{10}];
   is_id_switch = [is_id_switch; C{13}];
   class_idx = [class_idx; C{15}];
   has_overlap = [has_overlap; C{18}];
end

end

% Compute the segmentation scores.
function computeIntersectionOverUnion(pos_points, blob_points, other_pos_points, distance, distance_thresh, valid_indices)
% Compute the segmentation score as the intersection over union
% (see evaluation doc for explanation).
intersec_over_union = pos_points ./ (blob_points + other_pos_points);

% Find valid scores.
intersec_over_union_valid = intersec_over_union(valid_indices);

% Find scores for nearby objects.
nearby_indices = distance < distance_thresh; 
intersec_over_union_near = intersec_over_union(valid_indices & nearby_indices);

%Print the accuracy for segments at all distances.
fprintf('Intersection over union score: Accuracy: %s, Errors: %s, Mean accuracy: %s, mean errors: %s\n',  ...
    num2str(mean(intersec_over_union_valid > 0.5)), ...
    num2str(mean(intersec_over_union_valid <= 0.5)), ...
    num2str(mean(intersec_over_union_valid)), ...
    num2str(1 - mean(intersec_over_union_valid)));

% Print the accuracy for nearby segments.
fprintf('within %s m: Intersection over union score: Accuracy: %s, Errors: %s, Mean accuracy: %s, mean errors: %s\n',  ...
    num2str(distance_thresh), ...
    num2str(mean(intersec_over_union_near > 0.5)), ...
    num2str(mean(intersec_over_union_near <= 0.5)), ...
    num2str(mean(intersec_over_union_near)), ...
    num2str(1 - mean(intersec_over_union_near)));
end

% Compute whether our classification label for this object is correct.
% Types: the ground-truth label for this object (as a string)
% Class_index: our label for this object (as an index)
function incorrect_label = computeCorrectLabel(types, class_idx)
    incorrect_label = zeros(length(types), 1);

    % Our index "0" corresponds to the class label of Cyclist - 
    % check for errors in this class.
    incorrect_cyclist = strcmp(types, 'Cyclist') & class_idx ~= 0;
    incorrect_label = incorrect_label | incorrect_cyclist;

    % Our index "1" corresponds to the class labels of Car, Van, or Truck -
    % check for errors in these classes.
    incorrect_car = (strcmp(types, 'Car') | strcmp(types, 'Van') | strcmp(types, 'Truck')) & class_idx ~= 1;
    incorrect_label = incorrect_label | incorrect_car;
    
    % Our index "2" corresponds to the class labels of Pedestrian or Person sitting -
    % check for errors in these classes.
    incorrect_pedestrian = (strcmp(types, 'Pedestrian') | strcmp(types, 'Person_sitting')) & class_idx ~= 2;
    incorrect_label = incorrect_label | incorrect_pedestrian;
end

function valid_indices = getValidIndices(pos_points, has_overlap, types, class_idx, filter_incorrect_label)

% Because we are evaluating segmentation and not ground-estimation, 
% we ignore all segments that have 0 positive points
% (meaning that all points within the bounding box have been removed
% in the ground-estimation step
valid_indices = pos_points > 0;

% We ignore segments whose bounding box overlaps with another ground-truth
% bounding box, since in such cases the segmentation annotation is not sufficiently
% accurate to use.
valid_indices = valid_indices & ~has_overlap; 

% Compute whether we have classified each segment correctly.
incorrect_label = computeCorrectLabel(types, class_idx); 

% If the flag is set, only analyze segments that have been classified
% correctly (for debugging)
valid_indices = valid_indices &  (~filter_incorrect_label | ~incorrect_label);
end
