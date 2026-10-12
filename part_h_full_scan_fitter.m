function fitted_scan = part_h_full_scan_fitter(scan)
%PATH_H_FULL_SCAN_FITTER This function performs the parameter fitting for
%an entire set of wall partitions.  It also throws out bad partitions, both
%before and after performing the parameter fitting.  If shortcut labels are
%attached to the partitions, these get carried along with the partitions.

%% Unpacking the Struct
partition_count = scan.partition_count;
partitions_dirty = scan.partitions;
has_labels = isfield(scan,'shortcut_labels');
if has_labels
    labels_dirty = scan.shortcut_labels;
end

%% Throwing out the trash: Pre-Fit
partitions_cleaned_1 = {};
if has_labels
    labels_cleaned_1 = [];
end
clean_iter = 0;
for dirty_iter = 1:partition_count
    if pre_fit_trash_check(partitions_dirty{dirty_iter})
        clean_iter = clean_iter + 1;
        partitions_cleaned_1{clean_iter} = partitions_dirty{dirty_iter};
        if has_labels
            labels_cleaned_1(clean_iter) = labels_dirty(dirty_iter);
        end
    else
        partition_count = partition_count - 1;
    end
end

%% Performing the Fit
fits_dirty = cell(1,partition_count);
for fit_iter = 1:partition_count
    fits_dirty{fit_iter} = ...
        part_g_wall_param_fitter(partitions_cleaned_1{fit_iter});
end

%% Throwing out the trash: Post-Fit
partitions_cleaned_2 = {};
fits_cleaned = {};
if has_labels
    labels_cleaned_2 = [];
end
clean_iter = 0;
for dirty_iter = 1:partition_count
    if post_fit_trash_check(fits_dirty{dirty_iter})
        clean_iter = clean_iter + 1;
        partitions_cleaned_2{clean_iter} = ...
            partitions_cleaned_1{dirty_iter};
        fits_cleaned{clean_iter} = fits_dirty{dirty_iter};
        if has_labels
            labels_cleaned_2(clean_iter) = labels_cleaned_1(dirty_iter);
        end
    else
        partition_count = partition_count - 1;
    end
end

%% Packaging the Struct
fitted_scan.type = 'fitted_scan';
fitted_scan.time = scan.time;
fitted_scan.partitions = partitions_cleaned_2;
fitted_scan.partition_count = partition_count;
fitted_scan.fits = fits_cleaned;
if has_labels
    fitted_scan.shortcut_labels = labels_cleaned_2;
end
    
end

function is_ok = pre_fit_trash_check(scan_partition)

%% Unpacking the Struct
angle_count = scan_partition.angle_count;
angles = scan_partition.angles;
dists = scan_partition.dists;

%% Judgment
is_ok = ???;

end

function is_ok = post_fit_trash_check(wall_params)

%% Unpacking the Struct
meas = wall_params.meas;
r = wall_params.r;
alpha = wall_params.alpha;
covar = wall_params.covariance;

%% Judgment
is_ok = ???;

end