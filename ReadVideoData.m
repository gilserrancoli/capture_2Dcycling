function ReadVideoData
%% Authors: Gil Serrancolí and Peter Bogatikov
% This code reads a video and extracts a skeleton using OpenPose frame by 
% frame. The videos are supposed to be within the folders /videos/cam1 and
% /videos/cam2. 

close all;

%define global variables
global error_log frame;

%define options
Options.SecondFrameManual=1;
Options.Display=1;

%% Load video
current_folder=pwd;
Camera=2; %camera (1 or 2) to process
video_folder=['videos/cam ' num2str(Camera)]; %folder where the video is
name_file='HV-HR.mov'; %change the name according to your file
OP_folder='MEX_file';
main_folder=pwd;

cd(video_folder);

%Read the video
vid=VideoReader(name_file);
cd(current_folder);

%% number of frames to process
% nFrames=floor(vid.Duration*vid.FrameRate); %use it if you want to process
% all frames
nFrames=400;
frame=1; %initial frame to process

data=read(vid,frame);

%% Read skeleton using OpenPose
cd(OP_folder);
    [a b c pose(:,:,1) pose(:,:,2) pose(:,:,3)]=Process_Image_MEX(double(data));
cd(main_folder);
%choose only the skeleton which is at the centre of the image
size_data=size(data);
if size(pose,1)>1;
    if exist('aux_mat');
        clear aux_mat
    end
    pose_aux=pose(:,[10:15 20 22 23 25],1:2);
    aux_mat(1,1,1)=size_data(2)/2;
    aux_mat(1,1,2)=size_data(1)/2;
    aux_mat=repmat(aux_mat,size(pose_aux,1),10,1);
    sum_data=sum((aux_mat-pose_aux).^2,2);
    sum_data=sum(sum_data,3);
    
    [m I]=min(sum_data);
pose=pose(I,:,:);
end

[stats_5points{frame}, distinfo, I_hip, I_knee, I_ankle, I_front, I_heel]=PickFirstPoint_ind_OpenPose(data,pose);

close all;

%% Start to read the LED state 
% remove the syncroLED in case you use another way to syncrhonize the data
syncroLED=[];
fig=figure;
imshow(data);
title('Zoom in and press Enter or Press enter to place a rectangle around the syncro LED ');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('place a rectangle around the syncro LED');
rect=getrect(fig);
rectImage=data(rect(2):rect(2)+rect(4),rect(1):rect(1)+rect(3),:);
ledPixels=find(rectImage(:,:,1)==255&rectImage(:,:,2)==255&rectImage(:,:,3)==255, 1);
if isempty(ledPixels)
    syncroLED=0;
else
    syncroLED=1;
end
close all;

markers_in_columns=[stats_5points{frame}(1).Centroid stats_5points{frame}(2).Centroid stats_5points{frame}(3).Centroid stats_5points{frame}(4).Centroid stats_5points{frame}(5).Centroid];
time(1)=vid.CurrentTime;
foot_angle(frame)=calc_foot_angle(stats_5points{frame});
abs_angles(frame)=calc_abs_angles(stats_5points{frame});
delay=0;

%% start the for loop to read the skeleton frame by frame
frame0=frame;
for frame=(frame0+1):floor(nFrames)
     
    %% Get Data, first 2 frames manual
    manual=zeros(1,5);
    try
        data=read(vid,frame);
    catch
        break
    end
    %% Read Syncro
    rect=floor(rect);
    rectImage=data(rect(2):rect(2)+rect(4),rect(1):rect(1)+rect(3),:);
    ledPixels=find(rectImage(:,:,1)>150&rectImage(:,:,2)>220&rectImage(:,:,3)>245);
    if isempty(ledPixels);
        syncroLED(frame)=0;
    else
        syncroLED(frame)=1;
    end
    
    time(frame)=vid.CurrentTime-delay;
    imshow(data);
    if ((Options.SecondFrameManual==1)&(frame==2))
                if exist('pose');
                    clear pose;
                end
                cd(OP_folder);
                    [a b c pose(:,:,1) pose(:,:,2) pose(:,:,3)]=Process_Image_MEX(double(data));
                cd(main_folder);
                if size(pose,1)>1;
                    if exist('aux_mat');
                        clear aux_mat
                    end
                    pose_aux=pose(:,[10:15 20 22 23 25],1:2);
                    aux_mat(1,1,1)=size_data(2)/2;
                    aux_mat(1,1,2)=size_data(1)/2;
                    aux_mat=repmat(aux_mat,size(pose_aux,1),10,1);
                    sum_data=sum((aux_mat-pose_aux).^2,2);
                    sum_data=sum(sum_data(:,:,1),3);

                    [m I]=min(sum_data);
                pose=pose(I,:,:);
                end
                for i=1:5
                    switch i
                        case 1
                            cands=pose(1,[10 13],1:2);
                        case 2
                            cands=pose(1,[11 14],1:2);
                        case 3
                            cands=pose(1,[12 15],1:2);
                        case 4
                            cands=pose(1,[20 23],1:2);
                        case 5
                            cands=pose(1,[22 25],1:2);
                    end
                    alldist=calcDist_toAll_OpenPose(cands,stats_5points{1}(i).Centroid);
                    [m I]=min(alldist);
                    stats_aux=[];
                    stats_aux.Centroid=squeeze(cands(1,I,1:2))';
                    stats_5points{frame}(i)=stats_aux;
                end
        distance_segments(1,:)=Calc_dist_segments(stats_5points{frame-1});
        distance_segments(2,:)=Calc_dist_segments(stats_5points{frame});
        distance_segments_ref=mean(distance_segments);
        angles_vectors(1,:)=Calc_angles_vectors(stats_5points{frame-1});
        angles_vectors(2,:)=Calc_angles_vectors(stats_5points{frame});
        angles_vectors_ref=mean(angles_vectors);
    else
        
    end
    
    if stats_5points{end}(4).Centroid(1)>stats_5points{end}(5).Centroid(1)
        leg='right';
    elseif stats_5points{end}(4).Centroid(1)<=stats_5points{end}(5).Centroid(1)
        leg='left';
    end
    
    if frame>2   
        try
            if exist('pose');
                clear pose;
            end
            cd(OP_folder);
                [a b c pose(:,:,1) pose(:,:,2) pose(:,:,3)]=Process_Image_MEX(double(data));
            cd(main_folder);
            if size(pose,1)>1;
                if exist('aux_mat');
                    clear aux_mat
                end
                pose_aux=pose(:,[10:15 20 22 23 25],1:2);
                
                aux_mat(1,1,1)=size_data(2)/2;
                aux_mat(1,1,2)=size_data(1)/2;
%                 i=1;
%                 while i<=size(pose_aux,2)
%                     if any(pose_aux(:,i,:)==0)
%                         pose_aux(:,i,:)=[];
%                     end
%                     
%                     i=i+1;
%                 end
                np=size(pose_aux,2);
                aux_mat=repmat(aux_mat,size(pose_aux,1),np,1);
                sum_data=sum((aux_mat-pose_aux).^2,2);
                sum_data=sum(sum_data(:,:,1),3);

                [m I]=min(sum_data);
            pose=pose(I,:,:);
            end
        catch
            keyboard;
        end
        %%GA
         %% Look for hip;
         pose_hip=pose(:,[10 13],:);
        if frame==3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(1),pose_hip,stats_5points{frame-2}(1),time(frame-2:frame));
        elseif frame>3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(1),pose_hip,stats_5points{frame-2}(1),time(frame-3:frame),stats_5points{frame-3}(1));
        else
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(1),pose_hip);
        end
        [distcand I]=sort(alldist);
        Iaux=find(distcand<23);
        stats=[];
       
        if isempty(Iaux);
            Iaux=1:2;
            fprintf('hip point very far \n');
        end
        
        Icand=I(Iaux);
        if size(pose_hip,1)>1
            keyboard;
        end
        if exist('stats_hip_cand')
            clear stats_hip_cand;
        end
        for i=1:length(Icand)
            stats_hip_cand(i).Centroid=squeeze(pose_hip(1,Icand(i),1:2));
        end
        alldist_markers.hip=alldist(Icand);

        cand_hip=[];
        for k=1:length(stats_hip_cand)
            cand_hip(k,:)=[stats_hip_cand(k).Centroid(1:2)]; 
        end

        %% Look for knee
        pose_knee=pose(:,[11 14],:);
            if frame==3
                alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(2),pose_knee,stats_5points{frame-2}(2),time(frame-2:frame));
            elseif frame>3
                alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(2),pose_knee,stats_5points{frame-2}(2),time(frame-3:frame),stats_5points{frame-3}(2));
            else
                alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(2),pose_knee);
            end
            [distcand I]=sort(alldist);
            Iaux=find(distcand<27);
            usemarkers=0;

            if isempty(Iaux);
                Iaux=1:2;
                fprintf('knee point very far \n');
            end
            if usemarkers==0;
                Icand=I(Iaux);
                if size(pose_knee,1)>1
                    keyboard;
                end
                if exist('stats_knee_cand');
                    clear stats_knee_cand;
                end
                for i=1:length(Icand)
                    stats_knee_cand(i).Centroid=squeeze(pose_knee(1,Icand(i),1:2))';
                end
            else
                Icand=I(Iaux); 
                stats_knee_cand=stats(Icand); 
            end
            alldist_markers.knee=0;
            alldist_markers.knee=alldist(Icand);
        cand_knee=[];
        for k=1:length(stats_knee_cand)
            cand_knee(k,:)=[stats_knee_cand(k).Centroid(1:2)]; 
        end
        
        %% Look for ankle
        pose_ankle=pose(:,[12 15],:);
        if frame==3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(3),pose_ankle,stats_5points{frame-2}(3),time(frame-2:frame));
        elseif frame>3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(3),pose_ankle,stats_5points{frame-2}(3),time(frame-3:frame),stats_5points{frame-3}(3));
        else
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(3),pose_ankle);
        end
        [distcand I]=sort(alldist);
        Iaux=find(distcand<50);
        usemarkers=0;
        stats=[];
        if isempty(Iaux);
            Iaux=1;
            fprintf('ankle very far \n');
        end

        if isempty(Iaux);
            Iaux=1:2;
            fprintf('ankle point very far \n');
        end
        if usemarkers==0;
            Icand=I(Iaux);
            if size(pose_ankle,1)>1
                keyboard;
            end
            if exist('stats_ankle_cand');
                clear stats_ankle_cand;
            end
            for i=1:length(Icand)
                stats_ankle_cand(i).Centroid=squeeze(pose_ankle(1,Icand(i),1:2))';
            end
        else
            Icand=I(Iaux); 
            stats_ankle_cand=stats(Icand); 
        end
        alldist_markers.ankle=0;
        alldist_markers.ankle=alldist(Icand);

        cand_ankle=[];
        for k=1:length(stats_ankle_cand)
            cand_ankle(k,:)=[stats_ankle_cand(k).Centroid(1:2)]; 
        end
        % If the tibia length is weird, correct it  
        k=1;
        while k<=size(cand_ankle,1)
            try
                dist_tibia=norm(cand_ankle(k,:)-cand_knee(1,:));
            catch
                keyboard;
            end
            if abs((dist_tibia-distance_segments_ref(2))/distance_segments_ref(2))>0.2
               cand_ankle(end+1,:)=cand_knee(1,:)+(cand_ankle(1,:)-cand_knee(1,:))*distance_segments_ref(2)/dist_tibia;
               cand_ankle(k,:)=[];
            end
            k=k+1;
        end
        if frame>3
            dist_new_ankle=calcDist_ind_OpenPose(stats_5points{frame-1}(3),[1 cand_ankle(end,:)],stats_5points{frame-2}(3),time(frame-3:frame),stats_5points{frame-3}(3));
        elseif frame==3
            dist_new_ankle=calcDist_ind_OpenPose(stats_5points{frame-1}(3),[1 cand_ankle(end,:)],stats_5points{frame-2}(3),time(frame-2:frame));
        end
        alldist_markers.ankle(end+1)=dist_new_ankle;

        
        %% Look for front
        pose_front=pose(:,[23 20],:);
        if frame==3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(4),pose_front,stats_5points{frame-2}(4),time(frame-2:frame));
        elseif frame>3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(4),pose_front,stats_5points{frame-2}(4),time(frame-3:frame),stats_5points{frame-3}(4));
        else
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(4),pose_front);
        end
        [distcand I]=sort(alldist);
        Iaux=find(distcand<50);
        usemarkers=0;
        stats=[];
        
        if isempty(Iaux);
            Iaux=1:2;
            if isempty(find(distcand<50));
                pose_front(:)=0;
            else
                fprintf('front foot point very far \n');
            end
        end
        if usemarkers==0;

            Icand=I(Iaux);
            if size(pose_front,1)>1
                keyboard;
            end
            if exist('stats_front_cand');
                clear stats_front_cand;
            end
            try
                for i=1:length(Icand)
                    stats_front_cand(i).Centroid=squeeze(pose_front(1,Icand(i),1:2))';
                end
            catch
                keyboard;
            end
        else
            Icand=I(Iaux);
            stats_front_cand=stats(Icand);
        end
        alldist_markers.front=0;
        alldist_markers.front=alldist(Icand);

        cand_front=[];
        for k=1:length(stats_front_cand)
            cand_front(k,:)=[stats_front_cand(k).Centroid(1:2)]; 
        end
        
        %% Look for heel
        pose_heel=pose(:,[25 22],:);
        if frame==3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(5),pose_heel,stats_5points{frame-2}(5),time(frame-2:frame));
        elseif frame>3
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(5),pose_heel,stats_5points{frame-2}(5),time(frame-3:frame),stats_5points{frame-3}(5));
        else
            alldist=calcDist_ind_OpenPose(stats_5points{frame-1}(5),pose_heel);
        end
        [distcand I]=sort(alldist);
        Iaux=find(distcand<50);
        usemarkers=0;
        stats=[];
        
        if isempty(Iaux);
            Iaux=1:2;
            if isempty(find(distcand<50));
                pose_heel(:)=0;
            else
                fprintf('heel point very far \n');           
            end
        end
            
        if usemarkers==0;

            Icand=I(Iaux);
            if size(pose_heel,1)>1
                keyboard;
            end
            if exist('stats_heel_cand');
                clear stats_heel_cand;
            end
            for i=1:length(Icand)
                stats_heel_cand(i).Centroid=squeeze(pose_heel(1,Icand(i),1:2))';
            end
        else
            Icand=I(Iaux);
            stats_heel_cand=stats(Icand);
        end
        alldist_markers.heel=0;
        alldist_markers.heel=alldist(Icand);

        cand_heel=[];
        for k=1:length(stats_heel_cand)
            cand_heel(k,:)=[stats_heel_cand(k).Centroid(1:2)]; 
        end

                
        if any(cand_front(:)==0)|any(cand_heel(:)==0)
            if any(cand_front(:)==0)&(~any(cand_heel(:)==0))
                for i=1:size(cand_ankle,1)
                    for j=1:size(cand_heel,1);
                        stats_5points_aux=stats_5points;
                        stats_5points_aux{frame}(3).Centroid=cand_ankle(i,:);
                        stats_5points_aux{frame}(5).Centroid=cand_heel(j,:);
                        cand_front(end+1:end+3,:)=calculate_missing_point_OpenP(stats_5points_aux{frame},time,foot_angle,distance_segments_ref,4);
                        if ~isreal(cand_front)
                                cand_front(end-1,:)=(stats_5points_aux{frame}(3).Centroid+stats_5points_aux{frame}(5).Centroid)/2;
                                cand_front(end,:)=(stats_5points_aux{frame}(3).Centroid+stats_5points_aux{frame}(5).Centroid)/2;
                        end
                        for k=1:3
                            aux(k).Centroid=cand_front(end-3+k,:);
                        end
                        if frame==3
                            alldist=calcDist_ind(stats_5points{frame-1}(4),aux,stats_5points{frame-2}(4),time(frame-2:frame));
                        elseif frame>3
                            alldist=calcDist_ind(stats_5points{frame-1}(4),aux,stats_5points{frame-2}(4),time(frame-3:frame),stats_5points{frame-3}(4));
                        else
                            alldist=calcDist_ind(stats_5points{frame-1}(4),aux);
                        end
                        alldist_markers.front(end+1:end+3)=alldist;
                    end
                end
                error_log(frame)=1;
            elseif ~any(cand_front(:)==0)&(any(cand_heel(:)==0))
                for i=1:size(cand_ankle,1)
                    for j=1:size(cand_front,1);
                        stats_5points_aux=stats_5points;
                        stats_5points_aux{frame}(3).Centroid=cand_ankle(i,:);
                        stats_5points_aux{frame}(4).Centroid=cand_front(j,:);
                        try
                            cand_heel(end+1:end+3,:)=calculate_missing_point_OpenP(stats_5points_aux{frame},time,foot_angle,distance_segments_ref,5);
                            if ~isreal(cand_heel)
                                cand_heel(end-1,:)=(stats_5points_aux{frame}(3).Centroid+stats_5points_aux{frame}(4).Centroid)/2;
                                cand_heel(end,:)=(stats_5points_aux{frame}(3).Centroid+stats_5points_aux{frame}(4).Centroid)/2;
                            end
                        catch
                            keyboard;
                        end
                        for k=1:3
                            aux(k).Centroid=cand_heel(end-3+k,:);
                        end
                        if frame==3
                            alldist=calcDist_ind(stats_5points{frame-1}(5),aux,stats_5points{frame-2}(5),time(frame-2:frame));
                        elseif frame>3
                            alldist=calcDist_ind(stats_5points{frame-1}(5),aux,stats_5points{frame-2}(5),time(frame-3:frame),stats_5points{frame-3}(5));
                        else
                            alldist=calcDist_ind(stats_5points{frame-1}(5),aux);
                        end
                        alldist_markers.heel(end+1:end+3)=alldist;
                    end
                end                
                error_log(frame)=2;
            elseif (any(cand_hip(:)==0))&(any(cand_knee(:)==0))&(any(cand_front(:)==0))&(any(cand_heel(:)==0))
                try
                    if frame>20
                        cand_hip=Predict_fromEllipsoid(time,markers_in_columns,1);
                        cand_knee=Predict_fromEllipsoid(time,markers_in_columns,2);
                        cand_ankle=Predict_fromEllipsoid(time,markers_in_columns,3);
                        cand_front=Predict_fromEllipsoid(time,markers_in_columns,4);
                        cand_heel=Predict_fromEllipsoid(time,markers_in_columns,5);                    
                    else
                        cand_hip=Predict_fromPoly(time,markers_in_columns,1);
                        cand_knee=Predict_fromPoly(time,markers_in_columns,2);
                        cand_ankle=Predict_fromPoly(time,markers_in_columns,3);
                        cand_front=Predict_fromPoly(time,markers_in_columns,4);
                        cand_heel=Predict_fromPoly(time,markers_in_columns,5);
                    end
                catch
                    keyboard;
                end
            elseif any(cand_front(:)==0)&(any(cand_heel(:)==0))
                for i=1:size(cand_ankle,1)
                        stats_5points_aux=stats_5points;
                        stats_5points_aux{frame}(3).Centroid=cand_ankle(i,:);
                        try
                            [cand_front(end+1,:) cand_heel(end+1,:)]=calculate_missing_heel_front_OpenP(stats_5points_aux,time,foot_angle,distance_segments_ref);
                        catch
                            keyboard;
                        end
                        try
                            if exist('aux')
                                clear aux;
                            end
                            aux.Centroid=cand_front(end,:);
                        catch
                            keyboard;
                        end
                        if frame==3
                            alldist=calcDist_ind(stats_5points{frame-1}(4),aux,stats_5points{frame-2}(4),time(frame-2:frame));
                        elseif frame>3
                            alldist=calcDist_ind(stats_5points{frame-1}(4),aux,stats_5points{frame-2}(4),time(frame-3:frame),stats_5points{frame-3}(4));
                        else
                            alldist=calcDist_ind(stats_5points{frame-1}(4),aux);
                        end
                        alldist_markers.front(end+1)=alldist;
                        aux.Centroid=cand_heel(end,:);
                        if frame==3
                            alldist=calcDist_ind(stats_5points{frame-1}(5),aux,stats_5points{frame-2}(5),time(frame-2:frame));
                        elseif frame>3
                            alldist=calcDist_ind(stats_5points{frame-1}(5),aux,stats_5points{frame-2}(5),time(frame-3:frame),stats_5points{frame-3}(5));
                        else
                            alldist=calcDist_ind(stats_5points{frame-1}(5),aux);
                        end
                        alldist_markers.heel(end+1)=alldist;
                        
                end          
                error_log(frame)=3;
            end
        end
        try
            foot_angle_err=Calc_foot_angle_err(time,foot_angle,cand_heel,cand_front);
        catch
            keyboard;
        end
        
    try
        cand.hip=cand_hip;
        cand.knee=cand_knee;
        cand.ankle=cand_ankle;
        cand.front=cand_front;
        cand.heel=cand_heel;
    catch
        keyboard;
    end
        nm_hip=size(cand.hip,1);
        nm_knee=size(cand.knee,1);
        nm_ankle=size(cand.ankle,1);
        nm_front=size(cand.front,1);
        nm_heel=size(cand.heel,1);

        try
            if exist('table_scores')&exist('maxerr_l');
                clear table_scores;
                clear maxerr_l;
            end;
            for i1=1:nm_hip
                for i2=1:nm_knee
                    for i3=1:nm_ankle
                        for i4=1:nm_front
                            for i5=1:nm_heel
                                alldist_markers_aux.hip=alldist_markers.hip(i1);
                                alldist_markers_aux.knee=alldist_markers.knee(i2);
                                alldist_markers_aux.ankle=alldist_markers.ankle(i3);
                                alldist_markers_aux.front=alldist_markers.front(i4);
                                alldist_markers_aux.heel=alldist_markers.heel(i5);
                                [table_scores(i1,i2,i3,i4,i5), maxerr_l(i1,i2,i3,i4,i5)]=Calculate_score(cand.hip(i1,:),cand.knee(i2,:),cand.ankle(i3,:),cand.front(i4,:),cand.heel(i5,:),alldist_markers_aux,foot_angle_err(i4,i5),distance_segments_ref,angles_vectors_ref,1);
                            end
                        end
                    end
                end
            end
        catch
            keyboard;
        end
        try
            [sortdec I]=sort(table_scores(:));
        catch
            keyboard;
        end
        try
            [si1 si2 si3 si4 si5]=ind2sub(size(table_scores),I(1));
        catch
            keyboard;
        end
        stats_5points{frame}(1).Centroid=cand.hip(si1,:);
        stats_5points{frame}(2).Centroid=cand.knee(si2,:);
        stats_5points{frame}(3).Centroid=cand.ankle(si3,:);
        stats_5points{frame}(4).Centroid=cand.front(si4,:);
        stats_5points{frame}(5).Centroid=cand.heel(si5,:);
      
    end 
    try
        foot_angle(frame)=calc_foot_angle(stats_5points{frame});
    catch
        keyboard;
    end
    abs_angles(frame)=calc_abs_angles(stats_5points{frame});

    if Options.Display
        imshow(data);
        hold all;
        for k=1:5
            plot(stats_5points{frame}(k).Centroid(1),stats_5points{frame}(k).Centroid(2),'o','LineWidth',2);
        end
        title(['frame=' num2str(frame)]);
        pause(0.1);
    end
    try
        [stats_5points{frame}, foot_angle, abs_angles(frame)]=CheckFinalPointsFrame(stats_5points,foot_angle,distance_segments_ref,time,angles_vectors_ref,leg,abs_angles,markers_in_columns,data);
    catch
        keyboard;
    end
try
        markers_in_columns(frame,:)=[stats_5points{frame}(1).Centroid stats_5points{frame}(2).Centroid stats_5points{frame}(3).Centroid stats_5points{frame}(4).Centroid stats_5points{frame}(5).Centroid];
catch
    
    keyboard;
end

        if Options.Display
            imshow(data);
            hold all;
            for k=1:5
                plot(stats_5points{frame}(k).Centroid(1),stats_5points{frame}(k).Centroid(2),'o','LineWidth',2);
            end
            title(['frame=' num2str(frame)]);
            pause(0.1);
        end
        if 1==0;
            frame
            data=read(vid,frame);
            time(frame)=vid.CurrentTime-delay;
            [stats_5points{frame}]=PickFirstPoint_ind_OpenPose_manual(data);
            markers_in_columns(frame,:)=[stats_5points{frame}(1).Centroid stats_5points{frame}(2).Centroid stats_5points{frame}(3).Centroid stats_5points{frame}(4).Centroid stats_5points{frame}(5).Centroid];
            foot_angle(frame)=calc_foot_angle(stats_5points{frame});
            abs_angles(frame)=calc_abs_angles(stats_5points{frame});
        end
    disp(frame);
end

keyboard;
C=strsplit(video_folder,'/');
Subject_name=strrep(C(1),' ','');
trial=name_file(1:5);
cam_n=strrep(C(3),' ','');
eval([cam_n{1} '.markers_in_columns=markers_in_columns']);
eval([cam_n{1} '.syncroLED=syncroLED']);
eval([cam_n{1} '.time=time']);
eval([cam_n{1} '.error_log=error_log']);    

if exist([Subject_name{1} '_video_' trial '.mat']);
    save([Subject_name{1} '_video_' trial '.mat'],cam_n{1},'-append');
else
    save([Subject_name{1} '_video_' trial '.mat'],cam_n{1});
end


end

function [t_hip0, t_knee0,t_ankle0,t_front0,t_heel0] = SetInitThreshold (data)
        matlab_version_str=version;
        matlab_version=str2num(matlab_version_str(1:3));
        fig=figure;
        imshow(data);
        title('Zoom in to the HIP marker and press enter');
        zoom on;
        waitfor(gcf, 'CurrentCharacter', char(13));
        set (gcf,'CurrentCharacter','0')
        title('Click in the middle of the marker to set HIP threshold, and then enter');
        rgbValues=impixel;
        hsvValues=rgb2hsv(mean(rgbValues,1)./[255 255 255]);
        t_hip0.hueThresholdLow = hsvValues(:,1)-0.1;
        t_hip0.hueThresholdHigh = hsvValues(:,1)+0.1;
        t_hip0.saturationThresholdLow  = hsvValues(:,2)-0.1;
        t_hip0.saturationThresholdHigh = hsvValues(:,2)+0.1;
        t_hip0.valueThresholdLow = hsvValues(:,3)-0.1;
        t_hip0.valueThresholdHigh = hsvValues(:,3)+0.1;
        
        close all;
        imshow(data);
        title('Zoom in to the KNEE marker and press enter');
        zoom on;
        waitfor(gcf, 'CurrentCharacter', char(13));
        set (gcf,'CurrentCharacter','0')
        title('Click in the middle of the marker to set KNEE threshold, and then enter');
        manual_t_knee=ginput(1);
        rgbValues=impixel;
        hsvValues=rgb2hsv(mean(rgbValues,1)./[255 255 255]);
        zoom off;
        t_knee0.hueThresholdLow = hsvValues(:,1)-0.1;
        t_knee0.hueThresholdHigh = hsvValues(:,1)+0.1;
        t_knee0.saturationThresholdLow  = hsvValues(:,2)-0.1;
        t_knee0.saturationThresholdHigh = hsvValues(:,2)+0.1;
        t_knee0.valueThresholdLow = hsvValues(:,3)-0.1;
        t_knee0.valueThresholdHigh = hsvValues(:,3)+0.1;
        
        close all;
        imshow(data);
        title('Zoom in to the ANKLE marker and press enter');
        zoom on;
        waitfor(gcf, 'CurrentCharacter', char(13));
        set (gcf,'CurrentCharacter','0')
        title('Click in the middle of the marker to set ANKLE threshold');
        rgbValues=impixel;
        hsvValues=rgb2hsv(mean(rgbValues,1)./[255 255 255]);
        zoom off;
        t_ankle0.hueThresholdLow =hsvValues(:,1)-0.1;
        t_ankle0.hueThresholdHigh = hsvValues(:,1)+0.1;
        t_ankle0.saturationThresholdLow  = hsvValues(:,2)-0.1;
        t_ankle0.saturationThresholdHigh = hsvValues(:,2)+0.1;
        t_ankle0.valueThresholdLow = hsvValues(:,3)-0.1;
        t_ankle0.valueThresholdHigh = hsvValues(:,3)+0.1;
        
        close all;
        imshow(data);
        title('Zoom in to the FRONT FOOT marker and press enter');
        zoom on;
        waitfor(gcf, 'CurrentCharacter', char(13));
        set (gcf,'CurrentCharacter','0')
        title('Click in the middle of the marker to set FRONT FOOT threshold');
        rgbValues=impixel;
        hsvValues=rgb2hsv(mean(rgbValues,1)./[255 255 255]);
        zoom off;
        t_front0.hueThresholdLow = hsvValues(:,1)-0.1;
        t_front0.hueThresholdHigh = hsvValues(:,1)+0.1;
        t_front0.saturationThresholdLow  = hsvValues(:,2)-0.1;
        t_front0.saturationThresholdHigh = hsvValues(:,2)+0.1;
        t_front0.valueThresholdLow = hsvValues(:,3)-0.1;
        t_front0.valueThresholdHigh = hsvValues(:,3)+0.1;
        
        close all;
        imshow(data);
        title('Zoom in to the HEEL marker and press enter');
        zoom on;
        waitfor(gcf, 'CurrentCharacter', char(13));
        set (gcf,'CurrentCharacter','0')
        title('Click in the middle of the marker to set HEEL threshold');
        rgbValues=impixel;
        hsvValues=rgb2hsv(mean(rgbValues,1)./[255 255 255]);
        zoom off;
        t_heel0.hueThresholdLow = hsvValues(:,1)-0.1;
        t_heel0.hueThresholdHigh = hsvValues(:,1)+0.1;
        t_heel0.saturationThresholdLow  = hsvValues(:,2)-0.1;
        t_heel0.saturationThresholdHigh = hsvValues(:,2)+0.1;
        t_heel0.valueThresholdLow = hsvValues(:,3)-0.1;
        t_heel0.valueThresholdHigh = hsvValues(:,3)+0.1;
        
end   
function [t_front0,t_heel0] = SetInitThreshold_OpenPose (data)
        matlab_version_str=version;
        matlab_version=str2num(matlab_version_str(1:3));
        fig=figure;
        imshow(data);
        title('Zoom in to the FRONT FOOT marker and press enter');
        zoom on;
        waitfor(gcf, 'CurrentCharacter', char(13));
        set (gcf,'CurrentCharacter','0')
        title('Click in the middle of the marker to set FRONT FOOT threshold');
        rgbValues=impixel;
        hsvValues=rgb2hsv(mean(rgbValues,1)./[255 255 255]);
        zoom off;
        t_front0.hueThresholdLow = hsvValues(:,1)-0.1;
        t_front0.hueThresholdHigh = hsvValues(:,1)+0.1;
        t_front0.saturationThresholdLow  = hsvValues(:,2)-0.1;
        t_front0.saturationThresholdHigh = hsvValues(:,2)+0.1;
        t_front0.valueThresholdLow = hsvValues(:,3)-0.1;
        t_front0.valueThresholdHigh = hsvValues(:,3)+0.1;
        
        close all;
        imshow(data);
        title('Zoom in to the HEEL marker and press enter');
        zoom on;
        waitfor(gcf, 'CurrentCharacter', char(13));
        set (gcf,'CurrentCharacter','0')
        title('Click in the middle of the marker to set HEEL threshold');
        rgbValues=impixel;
        hsvValues=rgb2hsv(mean(rgbValues,1)./[255 255 255]);
        zoom off;
        t_heel0.hueThresholdLow = hsvValues(:,1)-0.1;
        t_heel0.hueThresholdHigh = hsvValues(:,1)+0.1;
        t_heel0.saturationThresholdLow  = hsvValues(:,2)-0.1;
        t_heel0.saturationThresholdHigh = hsvValues(:,2)+0.1;
        t_heel0.valueThresholdLow = hsvValues(:,3)-0.1;
        t_heel0.valueThresholdHigh = hsvValues(:,3)+0.1;
        
end 
function stats=LookForPoints(data,t,initranges)
    if exist('initranges')
    else
        initranges=[0 0];
    end
    % Convert RGB image to HSV
    hsvImage = rgb2hsv(data);
    % Extract out the H, S, and V images individually
    hImage = hsvImage(:,:,1);
    sImage = hsvImage(:,:,2);
    vImage = hsvImage(:,:,3);
    try
    % Now apply each color band's particular thresholds to the color band
    hueMask = (hImage >= t.hueThresholdLow) & (hImage <= t.hueThresholdHigh);
    saturationMask = (sImage >= t.saturationThresholdLow) & (sImage <= t.saturationThresholdHigh);
    valueMask = (vImage >= t.valueThresholdLow) & (vImage <= t.valueThresholdHigh);
    catch
        keyboard;
    end
    % Combine the masks to find where all 3 are "true."
    % Then we will have the mask of only the red parts of the image.
    coloredObjectsMask = uint8(hueMask & saturationMask & valueMask);

    %Smalles acceptable area to track
    smallestAcceptableArea = 1; % Keep areas only if they're bigger than this.

    % Get rid of small objects
    coloredObjectsMask = uint8(bwareaopen(coloredObjectsMask, smallestAcceptableArea));

     % You can only multiply integers if they are of the same type.
    % (coloredObjectsMask is a logical array.)
    % We need to convert the type of coloredObjectsMask to the same data type as hImage.
    coloredObjectsMask = cast(coloredObjectsMask, 'like', data); 

    % Use the colored object mask to mask out the colored-only portions of the rgb image.
    maskedImageR = coloredObjectsMask .* data(:,:,1);
    maskedImageG = coloredObjectsMask .* data(:,:,2);
    maskedImageB = coloredObjectsMask .* data(:,:,3);

%     imshow(maskedImageG);
    
     % Label all the connected components in the image.
    bw = bwlabel(coloredObjectsMask, 8);

    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(bw, 'BoundingBox', 'Centroid','Area');

    % Display the image
    imshow(data)
    
    hold on

    %This is a loop to bound the red objects in a rectangular box.
    for object = 1:length(stats)
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
%         rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
%         rectangle('Position',bb,'EdgeColor',[151 107 115]/255,'LineWidth',2)
        plot(bc(1),bc(2), '-m+')
        a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
        stats(object).BoundingBox=stats(object).BoundingBox+[initranges 0 0];
        stats(object).Centroid=stats(object).Centroid+initranges;
    end

    hold off
end

function [out distinfo]=PickFirstPoint(stats,data_video);

fig1=figure(1);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('get hip point');
manual_hip=ginput(1);
p_manual.Centroid=manual_hip;
dist_toHipPoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toHipPoint);
out(1)=stats(I);
ax=gca;
cla(ax);
distinfo(1,:)=[minv I];

imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('get knee point');
manual_knee=ginput(1);
p_manual.Centroid=manual_knee;
dist_toKneePoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toKneePoint);
out(2)=stats(I);
ax=gca;
cla(ax);
distinfo(2,:)=[minv I];

imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('get ankle point');
manual_ankle=ginput(1);
p_manual.Centroid=manual_ankle;
dist_toAnklePoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toAnklePoint);
out(3)=stats(I);
ax=gca;
cla(ax);
distinfo(3,:)=[minv I];

imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('get front foot point');
manual_front_foot=ginput(1);
p_manual.Centroid=manual_front_foot;
dist_toFrontFootPoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toFrontFootPoint);
out(4)=stats(I);
ax=gca;
cla(ax);
distinfo(4,:)=[minv I];

imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('get heel point');
manual_heel=ginput(1);
p_manual.Centroid=manual_heel;
dist_toHeelPoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toHeelPoint);
out(5)=stats(I);
ax=gca;
cla(ax);
distinfo(5,:)=[minv I];

end

function [out distinfo]=PickFirstPoint_ind(data_video,t);

fig1=figure(1);
stats=LookForPoints(data_video,t.t_hip);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('Zoom in and press Enter or Press enter to get hip point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get hip point');
manual_hip=ginput(1);
p_manual.Centroid=manual_hip;
dist_toHipPoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toHipPoint);
out(1)=stats(I);
ax=gca;
cla(ax);
distinfo(1,:)=[minv I];

stats=LookForPoints(data_video,t.t_knee);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('Zoom in and press Enter or Press enter to get knee point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get knee point');
manual_knee=ginput(1);
p_manual.Centroid=manual_knee;
dist_toKneePoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toKneePoint);
out(2)=stats(I);
ax=gca;
cla(ax);
distinfo(2,:)=[minv I];

stats=LookForPoints(data_video,t.t_ankle);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('Zoom in and press Enter or Press enter to get ankle point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get ankle point');
manual_ankle=ginput(1);
p_manual.Centroid=manual_ankle;
dist_toAnklePoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toAnklePoint);
out(3)=stats(I);
ax=gca;
cla(ax);
distinfo(3,:)=[minv I];

stats=LookForPoints(data_video,t.t_front);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('Zoom in and press Enter or Press enter to get foot front point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get front foot point');
manual_front_foot=ginput(1);
p_manual.Centroid=manual_front_foot;
dist_toFrontFootPoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toFrontFootPoint);
out(4)=stats(I);
ax=gca;
cla(ax);
distinfo(4,:)=[minv I];

stats=LookForPoints(data_video,t.t_heel);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:length(stats); 
    plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
end
title('Zoom in and press Enter or Press enter to get heel point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get heel point');
manual_heel=ginput(1);
p_manual.Centroid=manual_heel;
dist_toHeelPoint=calcDist_toAll(stats,p_manual);
[minv I]=min(dist_toHeelPoint);
out(5)=stats(I);
ax=gca;
cla(ax);
distinfo(5,:)=[minv I];

end

function [out, distinfo, I_hip, I_knee, I_ankle, I_front, I_heel]=PickFirstPoint_ind_OpenPose(data_video,pose);

%% Hip
fig1=figure(1);
% stats=LookForPoints(data_video,t.t_hip);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:size(pose,1);
    plot([pose(i,10,1) pose(i,11,1) pose(i,12,1) pose(i,25,1) pose(i,23,1) pose(i,12,1) ],...
         [pose(i,10,2) pose(i,11,2) pose(i,12,2) pose(i,25,2) pose(i,23,2) pose(i,12,2)],'LineWidth',2);
    plot([pose(i,13,1) pose(i,14,1) pose(i,15,1) pose(i,20,1) pose(i,22,1) pose(i,15,1)],...
         [pose(i,13,2) pose(i,14,2) pose(i,15,2) pose(i,20,2) pose(i,22,2) pose(i,15,2)],'LineWidth',2);
    plot(pose(i,10,1),pose(i,10,2),'o','LineWidth',2);
    plot(pose(i,11,1),pose(i,11,2),'o','LineWidth',2);
    plot(pose(i,12,1),pose(i,12,2),'o','LineWidth',2);
    plot(pose(i,13,1),pose(i,13,2),'o','LineWidth',2);
    plot(pose(i,14,1),pose(i,14,2),'o','LineWidth',2);
    plot(pose(i,15,1),pose(i,15,2),'o','LineWidth',2);
    plot(pose(i,20,1),pose(i,20,2),'o','LineWidth',2);
    plot(pose(i,22,1),pose(i,22,2),'o','LineWidth',2);
    plot(pose(i,23,1),pose(i,23,2),'o','LineWidth',2);
    plot(pose(i,25,1),pose(i,25,2),'o','LineWidth',2);
end
title('Zoom in and press Enter or Press enter to get hip point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get hip point');
manual_hip=ginput(1);
p_manual.Centroid=manual_hip;
dist_toHipPoint=calcDist_toAll_OpenPose(pose,p_manual);
if size(pose,1)==1;
    [minv I_hip]=min(dist_toHipPoint);
    out(1).Centroid=squeeze(pose(1,I_hip,1:2))';
else
    keyboard;
end
ax=gca;
cla(ax);
distinfo(1,:)=[minv I_hip];

%% Knee
% stats=LookForPoints(data_video,t.t_knee);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
for i=1:size(pose,1)
    plot([pose(i,10,1) pose(i,11,1) pose(i,12,1) pose(i,25,1) pose(i,23,1) pose(i,12,1) ],...
         [pose(i,10,2) pose(i,11,2) pose(i,12,2) pose(i,25,2) pose(i,23,2) pose(i,12,2)],'LineWidth',2);
    plot([pose(i,13,1) pose(i,14,1) pose(i,15,1) pose(i,20,1) pose(i,22,1) pose(i,15,1)],...
         [pose(i,13,2) pose(i,14,2) pose(i,15,2) pose(i,20,2) pose(i,22,2) pose(i,15,2)],'LineWidth',2);
    plot(pose(i,10,1),pose(i,10,2),'o','LineWidth',2);
    plot(pose(i,11,1),pose(i,11,2),'o','LineWidth',2);
    plot(pose(i,12,1),pose(i,12,2),'o','LineWidth',2);
    plot(pose(i,13,1),pose(i,13,2),'o','LineWidth',2);
    plot(pose(i,14,1),pose(i,14,2),'o','LineWidth',2);
    plot(pose(i,15,1),pose(i,15,2),'o','LineWidth',2);
    plot(pose(i,20,1),pose(i,20,2),'o','LineWidth',2);
    plot(pose(i,22,1),pose(i,22,2),'o','LineWidth',2);
    plot(pose(i,23,1),pose(i,23,2),'o','LineWidth',2);
    plot(pose(i,25,1),pose(i,25,2),'o','LineWidth',2);
end
title('Zoom in and press Enter or Press enter to get knee point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get knee point');
manual_knee=ginput(1);
p_manual.Centroid=manual_knee;
dist_toKneePoint=calcDist_toAll_OpenPose(pose,p_manual);
if size(pose,1)==1;
    [minv I_knee]=min(dist_toKneePoint);
    out(2).Centroid=squeeze(pose(1,I_knee,1:2))';
else
    keyboard;
end
ax=gca;
cla(ax);
distinfo(2,:)=[minv I_knee];

%% ankle
% stats=LookForPoints(data_video,t.t_ankle);
imshow(data_video);
hold all;
for i=1:size(pose,1)
    plot([pose(i,10,1) pose(i,11,1) pose(i,12,1) pose(i,25,1) pose(i,23,1) pose(i,12,1) ],...
         [pose(i,10,2) pose(i,11,2) pose(i,12,2) pose(i,25,2) pose(i,23,2) pose(i,12,2)],'LineWidth',2);
    plot([pose(i,13,1) pose(i,14,1) pose(i,15,1) pose(i,20,1) pose(i,22,1) pose(i,15,1)],...
         [pose(i,13,2) pose(i,14,2) pose(i,15,2) pose(i,20,2) pose(i,22,2) pose(i,15,2)],'LineWidth',2);
    plot(pose(i,10,1),pose(i,10,2),'o','LineWidth',2);
    plot(pose(i,11,1),pose(i,11,2),'o','LineWidth',2);
    plot(pose(i,12,1),pose(i,12,2),'o','LineWidth',2);
    plot(pose(i,13,1),pose(i,13,2),'o','LineWidth',2);
    plot(pose(i,14,1),pose(i,14,2),'o','LineWidth',2);
    plot(pose(i,15,1),pose(i,15,2),'o','LineWidth',2);
    plot(pose(i,20,1),pose(i,20,2),'o','LineWidth',2);
    plot(pose(i,22,1),pose(i,22,2),'o','LineWidth',2);
    plot(pose(i,23,1),pose(i,23,2),'o','LineWidth',2);
    plot(pose(i,25,1),pose(i,25,2),'o','LineWidth',2);
end
title('Zoom in and press Enter or Press enter to get ankle point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get ankle point');
manual_ankle=ginput(1);
p_manual.Centroid=manual_ankle;
dist_toAnklePoint=calcDist_toAll_OpenPose(pose,p_manual);
if size(pose,1)==1;
    [minv I_ankle]=min(dist_toAnklePoint);
    out(3).Centroid=squeeze(pose(1,I_ankle,1:2))';
else
    keyboard;
end
ax=gca;
cla(ax);
distinfo(3,:)=[minv I_ankle];

%% Front
% stats=LookForPoints(data_video,t.t_front);
imshow(data_video);
hold all;
for i=1:size(pose,1)
    plot([pose(i,10,1) pose(i,11,1) pose(i,12,1) pose(i,25,1) pose(i,23,1) pose(i,12,1) ],...
         [pose(i,10,2) pose(i,11,2) pose(i,12,2) pose(i,25,2) pose(i,23,2) pose(i,12,2)],'LineWidth',2);
    plot([pose(i,13,1) pose(i,14,1) pose(i,15,1) pose(i,20,1) pose(i,22,1) pose(i,15,1)],...
         [pose(i,13,2) pose(i,14,2) pose(i,15,2) pose(i,20,2) pose(i,22,2) pose(i,15,2)],'LineWidth',2);
    plot(pose(i,10,1),pose(i,10,2),'o','LineWidth',2);
    plot(pose(i,11,1),pose(i,11,2),'o','LineWidth',2);
    plot(pose(i,12,1),pose(i,12,2),'o','LineWidth',2);
    plot(pose(i,13,1),pose(i,13,2),'o','LineWidth',2);
    plot(pose(i,14,1),pose(i,14,2),'o','LineWidth',2);
    plot(pose(i,15,1),pose(i,15,2),'o','LineWidth',2);
    plot(pose(i,20,1),pose(i,20,2),'o','LineWidth',2);
    plot(pose(i,22,1),pose(i,22,2),'o','LineWidth',2);
    plot(pose(i,23,1),pose(i,23,2),'o','LineWidth',2);
    plot(pose(i,25,1),pose(i,25,2),'o','LineWidth',2);
end
title('Zoom in and press Enter or Press enter to get foot front point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get front foot point');
manual_front_foot=ginput(1);
p_manual.Centroid=manual_front_foot;
dist_toFrontFootPoint=calcDist_toAll_OpenPose(pose,p_manual);
if size(pose,1)==1;
    [minv I_front]=min(dist_toFrontFootPoint);
    out(4).Centroid=squeeze(pose(1,I_front,1:2))';
else
    keyboard;
end
ax=gca;
cla(ax);
distinfo(4,:)=[minv I_front];

%% Heel
% stats=LookForPoints(data_video,t.t_heel);
imshow(data_video);
hold all;
for i=1:size(pose,1)
    plot([pose(i,10,1) pose(i,11,1) pose(i,12,1) pose(i,25,1) pose(i,23,1) pose(i,12,1) ],...
         [pose(i,10,2) pose(i,11,2) pose(i,12,2) pose(i,25,2) pose(i,23,2) pose(i,12,2)],'LineWidth',2);
    plot([pose(i,13,1) pose(i,14,1) pose(i,15,1) pose(i,20,1) pose(i,22,1) pose(i,15,1)],...
         [pose(i,13,2) pose(i,14,2) pose(i,15,2) pose(i,20,2) pose(i,22,2) pose(i,15,2)],'LineWidth',2);
    plot(pose(i,10,1),pose(i,10,2),'o','LineWidth',2);
    plot(pose(i,11,1),pose(i,11,2),'o','LineWidth',2);
    plot(pose(i,12,1),pose(i,12,2),'o','LineWidth',2);
    plot(pose(i,13,1),pose(i,13,2),'o','LineWidth',2);
    plot(pose(i,14,1),pose(i,14,2),'o','LineWidth',2);
    plot(pose(i,15,1),pose(i,15,2),'o','LineWidth',2);
    plot(pose(i,20,1),pose(i,20,2),'o','LineWidth',2);
    plot(pose(i,22,1),pose(i,22,2),'o','LineWidth',2);
    plot(pose(i,23,1),pose(i,23,2),'o','LineWidth',2);
    plot(pose(i,25,1),pose(i,25,2),'o','LineWidth',2);
end
title('Zoom in and press Enter or Press enter to get heel point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get heel point');
manual_heel=ginput(1);
p_manual.Centroid=manual_heel;
dist_toHeelPoint=calcDist_toAll_OpenPose(pose,p_manual);
if size(pose,1)==1;
    [minv I_heel]=min(dist_toHeelPoint);
    out(5).Centroid=squeeze(pose(1,I_heel,1:2))';
else
    keyboard;
end
ax=gca;
cla(ax);
distinfo(5,:)=[minv I_heel];

end

function [out]=PickFirstPoint_ind_OpenPose_manual(data_video);

%% Hip
fig1=figure(1);
% stats=LookForPoints(data_video,t.t_hip);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
title('Zoom in and press Enter or Press enter to get hip point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get hip point');
manual_hip=ginput(1);
p_manual=manual_hip;
out(1).Centroid=p_manual;

%% Knee
% stats=LookForPoints(data_video,t.t_knee);
imshow(data_video);
hold all;
% ax=gca;
% XLIM=ax.XLIM;
% YLIM=ax.YLIM;
title('Zoom in and press Enter or Press enter to get knee point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get knee point');
manual_knee=ginput(1);
p_manual=manual_knee;
out(2).Centroid=p_manual;

%% ankle
% stats=LookForPoints(data_video,t.t_ankle);
imshow(data_video);
hold all;
title('Zoom in and press Enter or Press enter to get ankle point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get ankle point');
manual_ankle=ginput(1);
p_manual=manual_ankle;
out(3).Centroid=p_manual;

%% Front
% stats=LookForPoints(data_video,t.t_front);
imshow(data_video);
hold all;
title('Zoom in and press Enter or Press enter to get foot front point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get front foot point');
manual_front_foot=ginput(1);
p_manual=manual_front_foot;
out(4).Centroid=p_manual;

%% Heel
% stats=LookForPoints(data_video,t.t_heel);
imshow(data_video);
hold all;
title('Zoom in and press Enter or Press enter to get heel point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
title('get heel point');
manual_heel=ginput(1);
p_manual=manual_heel;
out(5).Centroid=p_manual;

end

function alldist=calcDist_toAll(s1,s2);
for i=1:length(s1)
    for j=1:length(s2);
        alldist(i,j)=norm(s1(i).Centroid-s2(j).Centroid);
    end
end
end

function alldist=calcDist_toAll_OpenPose(s1,s2);
    for i=1:size(s1,1)
        for k=1:size(s1,2);
            if isstruct(s2)
                for j=1:length(s2);
                    alldist(i,j,k)=norm(squeeze(s1(i,k,1:2))'-s2(j).Centroid);
                end
            else
                for j=1:size(s2,1);
                    alldist(i,j,k)=norm(squeeze(s1(i,k,1:2))'-s2(i,:));
                end
            end
        end
    end
end

function [t_hip, t_knee, t_ankle, t_front, t_heel]=getHSVvalues(data,stats_5points)
 % Convert RGB image to HSV
    hsvImage = rgb2hsv(data);
    % Extract out the H, S, and V images individually
    hImage = hsvImage(:,:,1);
    sImage = hsvImage(:,:,2);
    vImage = hsvImage(:,:,3);
    imshow(hImage);
    t_hip=findThresholds(round(stats_5points(1).Centroid),data);
    t_knee=findThresholds(round(stats_5points(2).Centroid),data);
    t_ankle=findThresholds(round(stats_5points(3).Centroid),data);
    t_front=findThresholds(round(stats_5points(4).Centroid),data);
    t_heel=findThresholds(round(stats_5points(5).Centroid),data);
end

function t=findThresholds(Centroid,data);
    hsvImage = rgb2hsv(data);
    % Extract out the H, S, and V images individually
    hImage = hsvImage(:,:,1);
    sImage = hsvImage(:,:,2);
    vImage = hsvImage(:,:,3);
    delta=0.1;
    
    t.hueCentral=hImage(Centroid(2),Centroid(1));
    t.hueThresholdLow=t.hueCentral-delta/2;
    t.hueThresholdHigh=t.hueCentral+delta/2;
    
    t.saturationCentral=sImage(Centroid(2),Centroid(1));
    t.saturationThresholdLow=t.saturationCentral-delta;
    t.saturationThresholdHigh=t.saturationCentral+delta;
    
    t.valueCentral=vImage(Centroid(2),Centroid(1));
    t.valueThresholdLow=t.valueCentral-delta;
    t.valueThresholdHigh=t.valueCentral+delta;
end
function alldist=calcDist(s1,s2);
    for i=1:5
        for j=1:length(s2);
            alldist(i,j)=norm(s1(i).Centroid-s2(j).Centroid);
        end
    end
end
function alldist=calcDist_ind(s1,s2,s1_m1,time,s1_m2);

if exist('s1_m2');
    tm1=time(1);
    t0=time(2);
    t1=time(3);
    t2=time(4);
    x=[s1_m2.Centroid(1); s1_m1.Centroid(1); s1.Centroid(1)];
    y=[s1_m2.Centroid(2); s1_m1.Centroid(2); s1.Centroid(2)];
    fx=fit(time(1:3)',x,'smoothingspline');
    fy=fit(time(1:3)',y,'smoothingspline');
    x_fit=fx(time(4));
    y_fit=fy(time(4));
    
    for j=1:length(s2);
        vm1=(s1_m1.Centroid-s1_m2.Centroid)/(t0-tm1);
        v0=(s1.Centroid-s1_m1.Centroid)/(t1-t0);
        a0=(v0-vm1)/(t1-t0);
%         alldist(j)=norm(s1.Centroid+v0*(t2-t1)+(1/2)*a0*((t2-t1)^2)-s2(j).Centroid);
        alldist(j)=norm([x_fit y_fit]-s2(j).Centroid);
    end
    
else
    if exist('s1_m1');
        t0=time(1);
        t1=time(2);
        t2=time(3);
        x=[s1_m1.Centroid(1); s1.Centroid(1)];
        y=[s1_m1.Centroid(2); s1.Centroid(2)];
        fx=fit(time(1:2)',x,'smoothingspline');
        fy=fit(time(1:2)',y,'smoothingspline');
        x_fit=fx(time(3));
        y_fit=fy(time(3));
        for j=1:length(s2);
            v0=(s1.Centroid-s1_m1.Centroid)/(t1-t0);
%             alldist(j)=norm(s1.Centroid+v0*(t2-t1)-s2(j).Centroid);
            alldist(j)=norm([x_fit y_fit]-s2(j).Centroid);

        end
    else
        for j=1:length(s2);
            alldist(j)=norm(s1.Centroid-s2(j).Centroid);
        end
    end
end

end
function alldist=calcDist_ind_OpenPose(s1,s2,s1_m1,time,s1_m2);
%s2 is pose
if exist('s1_m2');
    tm1=time(1);
    t0=time(2);
    t1=time(3);
    t2=time(4);
    x=[s1_m2.Centroid(1); s1_m1.Centroid(1); s1.Centroid(1)];
    y=[s1_m2.Centroid(2); s1_m1.Centroid(2); s1.Centroid(2)];
    fx=fit(time(1:3)',x,'smoothingspline');
    fy=fit(time(1:3)',y,'smoothingspline');
    x_fit=fx(time(4));
    y_fit=fy(time(4));
    if size(s2,1)>1
        keyboard;
    end
    if size(s2,3)==3;
        L=size(s2,2);
    else 
        L=size(s2,1);
    end
    for j=1:L;
        vm1=(s1_m1.Centroid-s1_m2.Centroid)/(t0-tm1);
        v0=(s1.Centroid-s1_m1.Centroid)/(t1-t0);
        a0=(v0-vm1)/(t1-t0);
%         alldist(j)=norm(s1.Centroid+v0*(t2-t1)+(1/2)*a0*((t2-t1)^2)-s2(j).Centroid);
try
        if L>1 
            alldist(j)=norm([x_fit y_fit]-squeeze(s2(1,j,1:2))');
        else
            alldist(j)=norm([x_fit y_fit]-s2(2:3));
        end
catch
    keyboard;
end
    end
    
else
    if exist('s1_m1');
        t0=time(1);
        t1=time(2);
        t2=time(3);
        x=[s1_m1.Centroid(1); s1.Centroid(1)];
        y=[s1_m1.Centroid(2); s1.Centroid(2)];
        fx=fit(time(1:2)',x,'smoothingspline');
        fy=fit(time(1:2)',y,'smoothingspline');
        x_fit=fx(time(3));
        y_fit=fy(time(3));
        if size(s2,1)>1
            keyboard;
        end
        if size(s2,3)==3;
            L=size(s2,2);
        else 
            L=size(s2,1);
        end
        for j=1:L;
            v0=(s1.Centroid-s1_m1.Centroid)/(t1-t0);
%             alldist(j)=norm(s1.Centroid+v0*(t2-t1)-s2(j).Centroid);
try
            if L>1 
                alldist(j)=norm([x_fit y_fit]-squeeze(s2(1,j,1:2))');
            else
                alldist(j)=norm([x_fit y_fit]-s2(2:3));
            end
catch
    keyboard;
end
        end
    else
        for j=1:size(s2,2);
            alldist(j)=norm(s1.Centroid-squeeze(s2(1,j,1:2)));
        end
    end
end

end


function out=PickManualPoint(stats,data_video,pos_error);
    for i=1:length(stats);
       out(i)=stats(i); 
    end
    
    if any(1==pos_error);
        fig1=figure(1);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        for i=1:length(stats); 
            plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
        end
        title('Zoom in and press Enter or Press enter to get hip point');
% use mouse button to zoom in or out
% Press Enter to get out of the zoom mode
zoom on;
% Wait for the most recent key to become the return/enter key
waitfor(gcf, 'CurrentCharacter', char(13));
zoom reset;
zoom off;
set (gcf,'CurrentCharacter','0')
        title('get hip point');
        manual_hip=ginput(1);
        p_manual.Centroid=manual_hip;
        out(1).Centroid=manual_hip;
        ax=gca;
        cla(ax);
    end
    if any(2==pos_error)
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        for i=1:length(stats); 
            plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
        end
        title('Zoom in and press Enter or Press enter to get knee point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get knee point');
        manual_knee=ginput(1);
        p_manual.Centroid=manual_knee;
        out(2).Centroid=manual_knee;
        ax=gca;
        cla(ax);
    end
    if any(3==pos_error);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        for i=1:length(stats); 
            plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
        end
        title('Zoom in and press Enter or Press enter to get ankle point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get ankle point');
        manual_ankle=ginput(1);
        p_manual.Centroid=manual_ankle;
        out(3).Centroid=manual_ankle;
        ax=gca;
        cla(ax);
    end
    if any(4==pos_error);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        for i=1:length(stats); 
            plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
        end
        title('Zoom in and press Enter or Press enter to get front foot point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get front foot point');
        manual_front_foot=ginput(1);
        p_manual.Centroid=manual_front_foot;
        out(4).Centroid=manual_front_foot;
        ax=gca;
        cla(ax);
    end
    if any(5==pos_error);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        for i=1:length(stats); 
            plot(stats(i).Centroid(1),stats(i).Centroid(2),'o');
        end
        title('Zoom in and press Enter or Press enter to get heel point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get heel point');
        manual_heel=ginput(1);
        p_manual.Centroid=manual_heel;
        out(5).Centroid=manual_heel;
        ax=gca;
        cla(ax);
    end
end
function out=PickManualPoint_OpenPose(pose,data_video,pos_error);
%     for i=1:length(stats);
%        out(i)=stats(i); 
%     end
    
    if any(1==pos_error);
        fig1=figure(1);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        if size(pose,1)>1
            keyboard;
        end
        for i=1:size(pose,1); 
            plot(pose(1,i,1),pose(1,i,2),'o');
        end
        title('Zoom in and press Enter or Press enter to get hip point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get hip point');
        manual_hip=ginput(1);
        p_manual.Centroid=manual_hip;
        out(1).Centroid=manual_hip;
        ax=gca;
        cla(ax);
    end
    if any(2==pos_error)
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        if size(pose,1)>1
            keyboard;
        end
        for i=1:size(pose,1); 
            plot(pose(1,i,1),pose(1,i,2),'o');
        end
        title('Zoom in and press Enter or Press enter to get knee point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get knee point');
        manual_knee=ginput(1);
        p_manual.Centroid=manual_knee;
        out(2).Centroid=manual_knee;
        ax=gca;
        cla(ax);
    end
    if any(3==pos_error);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        if size(pose,1)>1
            keyboard;
        end
        for i=1:size(pose,1); 
            plot(pose(1,i,1),pose(1,i,2),'o');
        end
        title('Zoom in and press Enter or Press enter to get ankle point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get ankle point');
        manual_ankle=ginput(1);
        p_manual.Centroid=manual_ankle;
        out(3).Centroid=manual_ankle;
        ax=gca;
        cla(ax);
    end
    if any(4==pos_error);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        if size(pose,1)>1
            keyboard;
        end
        for i=1:size(pose,1); 
            plot(pose(1,i,1),pose(1,i,2),'o');
        end
        title('Zoom in and press Enter or Press enter to get front foot point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get front foot point');
        manual_front_foot=ginput(1);
        p_manual.Centroid=manual_front_foot;
        out(4).Centroid=manual_front_foot;
        ax=gca;
        cla(ax);
    end
    if any(5==pos_error);
        imshow(data_video);
        hold all;
        % ax=gca;
        % XLIM=ax.XLIM;
        % YLIM=ax.YLIM;
        if size(pose,1)>1
            keyboard;
        end
        for i=1:size(pose,1); 
            plot(pose(1,i,1),pose(1,i,2),'o');
        end
        title('Zoom in and press Enter or Press enter to get heel point');
        % use mouse button to zoom in or out
        % Press Enter to get out of the zoom mode
        zoom on;
        % Wait for the most recent key to become the return/enter key
        waitfor(gcf, 'CurrentCharacter', char(13));
        zoom reset;
        zoom off;
        set (gcf,'CurrentCharacter','0')
        title('get heel point');
        manual_heel=ginput(1);
        p_manual.Centroid=manual_heel;
        out(5).Centroid=manual_heel;
        ax=gca;
        cla(ax);
    end
end
function distance_segments=Calc_dist_segments(stats_5points);
    distance_segments=[norm(stats_5points(2).Centroid-stats_5points(1).Centroid),... %femur
        norm(stats_5points(2).Centroid-stats_5points(3).Centroid),... %tibia
        norm(stats_5points(3).Centroid-stats_5points(4).Centroid),... %ankle-tip toe
        norm(stats_5points(3).Centroid-stats_5points(5).Centroid),... %ankle-heel
        norm(stats_5points(4).Centroid-stats_5points(5).Centroid)];   %heel-tip toe
end
function angles_vectors=Calc_angles_vectors(stats_5points);
    l_ankle_heel=stats_5points(5).Centroid-stats_5points(3).Centroid;
    l_ankle_tiptoe=stats_5points(4).Centroid-stats_5points(3).Centroid;
    l_heel_tiptoe=stats_5points(4).Centroid-stats_5points(5).Centroid;
    angles_vectors=[acos((sum(l_ankle_heel.*l_ankle_tiptoe)/(norm(l_ankle_heel)*norm(l_ankle_tiptoe)))),...
        acos((sum(l_ankle_heel.*l_heel_tiptoe)/(norm(l_ankle_heel)*norm(l_heel_tiptoe)))),...
        acos((sum(l_ankle_tiptoe.*l_heel_tiptoe)/(norm(l_ankle_tiptoe)*norm(l_heel_tiptoe))))];
end
function indx=FindAppropriatePoint(stats_5points,stats,sortdist,I,distance_segments_ref,indx_segment,weight,elips)
    if exist('weight')
        if isempty(weight)
            w=1;
        else
            w=weight;
        end
    else
        w=1;
    end
    distm45=find(sortdist<45);
    for i=1:length(distm45);
        if indx_segment==4
            indx_initialpoint=3;
        else
            indx_initialpoint=indx_segment;
        end
        dist_segment(i)=sqrt(sum((stats(I(i)).Centroid-stats_5points(indx_initialpoint).Centroid).^2));
    end
    error_dist=dist_segment-distance_segments_ref(indx_segment);
    fdecision=error_dist.^2+w*sortdist(distm45).^2;
    [m, Iout]=min(fdecision);
    indx=I(Iout);
end

function [dist I]=DetectPreviousSimilarFrame(markers_in_columns);
    nf=size(markers_in_columns,1);
    if nf<20
        dist=[];
        I=[];
    else
        [dist I]=sort(sqrt(abs((markers_in_columns(1:(end-5),5)-markers_in_columns(end,5)).^2+(markers_in_columns(1:(end-5),6)-markers_in_columns(end,6)).^2)));
        if dist(1)>5
            dist=[];
            I=[];
        end
    end
end

function [score, maxerror]=Calculate_score(hip,knee,ankle,front,heel,alldist_markers,foot_angle_err,distance_segments_ref,angles_vectors_ref,w_cnstlengths)
dist_femur=sqrt(sum((hip-knee).^2));
dist_tibia=sqrt(sum((knee-ankle).^2));
dist_ankletoe=sqrt(sum((ankle-front).^2));
dist_ankleheel=sqrt(sum((ankle-heel).^2));
dist_heeltoe=sqrt(sum((heel-front).^2));



err_femur=dist_femur-distance_segments_ref(1);
err_tibia=dist_tibia-distance_segments_ref(2);
if any(front==0)
    err_ankletoe=1000;
else
    err_ankletoe=dist_ankletoe-distance_segments_ref(3);
end
if any(heel==0)
    err_ankleheel=1000;
else
    err_ankleheel=dist_ankleheel-distance_segments_ref(4);
end
if any(front==0)|any(heel==0)
    err_heeltoe=1000;    
else
    err_heeltoe=dist_heeltoe-distance_segments_ref(5);
end

stats_5points(1).Centroid=hip;
stats_5points(2).Centroid=knee;
stats_5points(3).Centroid=ankle;
stats_5points(4).Centroid=front;
stats_5points(5).Centroid=heel;
angles_vectors=Calc_angles_vectors(stats_5points);

score=w_cnstlengths*[err_femur^2+err_tibia^2+...
           err_ankletoe^2+err_ankleheel^2+err_heeltoe^2]+...
       [sum(((angles_vectors-angles_vectors_ref)*(180/pi)/5).^2)]+...    
       [alldist_markers.hip^2+alldist_markers.knee^2+...
           alldist_markers.ankle^2+alldist_markers.front^2+...
           alldist_markers.heel^2]+10*foot_angle_err^2;

%make imposible the case when both heel and tiptoe are in a higher position than the
%ankle 
if (heel(2)<ankle(2))&(front(2)<ankle(2))
    score=inf;
%make imposible the case when both heel and tiptoe are in a higher position than the
%ankle     
elseif (heel(1)>front(1))&(heel(2)<ankle(2))&(heel(1)<ankle(1))
    score=inf;
end
       
[maxerror.m, maxerror.I]=sort(abs([err_femur/distance_segments_ref(1),err_tibia/distance_segments_ref(2),err_ankletoe/distance_segments_ref(3),err_ankleheel/distance_segments_ref(4),err_heeltoe/distance_segments_ref(5)]),'descend');

end
function scores=fun(x,cand,distance_segments_ref,alldist_markers)
    cand_hip=cand.hip;
    cand_knee=cand.knee;
    cand_ankle=cand.ankle;
    cand_front=cand.front;
    cand_heel=cand.heel;

    
    for j=1:size(x,1)
        p=x{j};
        dist_femur=sqrt(sum((cand_hip(p(1),:)-cand_knee(p(2),:)).^2));
        dist_tibia=sqrt(sum((cand_knee(p(2),:)-cand_ankle(p(3),:)).^2));
        dist_ankletoe=sqrt(sum((cand_ankle(p(3),:)-cand_front(p(4),:)).^2));
        dist_ankleheel=sqrt(sum((cand_ankle(p(3),:)-cand_heel(p(5),:)).^2));
        f=[(dist_femur-distance_segments_ref(1))^2+...
           (dist_tibia-distance_segments_ref(2))^2+...
           (dist_ankletoe-distance_segments_ref(3))^2+...
           (dist_ankleheel-distance_segments_ref(4))^2]+...
           [alldist_markers.hip(p(1))^2+alldist_markers.knee(p(2))^2+...
           alldist_markers.ankle(p(3))^2+alldist_markers.front(p(4))^2+...
           alldist_markers.heel(p(5))^2];
        scores(j)=f;
    end
end

function pop = create_permutations(NVARS,FitnessFcn,options)
    global cand;
    n=NVARS;
    totalPopulationSize = sum(options.PopulationSize);
    pop=cell(totalPopulationSize,1);
    for i=1:totalPopulationSize;
       pop{i}=[randi(size(cand.hip,1)) randi(size(cand.knee,1)) randi(size(cand.ankle,1)) randi(size(cand.front,1)) randi(size(cand.heel,1))];
    end
end

function xoverKids  = crossover_permutation(parents,options,NVARS, ...
    FitnessFcn,thisScore,thisPopulation)
%   CROSSOVER_PERMUTATION Custom crossover function for traveling salesman.
%   XOVERKIDS = CROSSOVER_PERMUTATION(PARENTS,OPTIONS,NVARS, ...
%   FITNESSFCN,THISSCORE,THISPOPULATION) crossovers PARENTS to produce
%   the children XOVERKIDS.
%
%   The arguments to the function are 
%     PARENTS: Parents chosen by the selection function
%     OPTIONS: Options created from OPTIMOPTIONS
%     NVARS: Number of variables 
%     FITNESSFCN: Fitness function 
%     STATE: State structure used by the GA solver 
%     THISSCORE: Vector of scores of the current population 
%     THISPOPULATION: Matrix of individuals in the current population
nKids = length(parents)/2;
xoverKids = cell(nKids,1); % Normally zeros(nKids,NVARS);
index = 1;

for i=1:nKids
    % here is where the special knowledge that the population is a cell
    % array is used. Normally, this would be thisPopulation(parents(index),:);
    parent = thisPopulation{parents(index)};
    index = index + 2;

    % Flip a section of parent1.
    p1 = ceil((length(parent) -1) * rand);
    p2 = p1 + ceil((length(parent) - p1- 1) * rand);
    child = parent;
    child(p1:p2) = fliplr(child(p1:p2));
    xoverKids{i} = child; % Normally, xoverKids(i,:);
end

end

function mutationChildren = mutate_permutation(parents ,options,NVARS, ...
    FitnessFcn, state, thisScore,thisPopulation,mutationRate)
%   MUTATE_PERMUTATION Custom mutation function for traveling salesman.
%   MUTATIONCHILDREN = MUTATE_PERMUTATION(PARENTS,OPTIONS,NVARS, ...
%   FITNESSFCN,STATE,THISSCORE,THISPOPULATION,MUTATIONRATE) mutate the
%   PARENTS to produce mutated children MUTATIONCHILDREN.
%
%   The arguments to the function are 
%     PARENTS: Parents chosen by the selection function
%     OPTIONS: Options created from OPTIMOPTIONS
%     NVARS: Number of variables 
%     FITNESSFCN: Fitness function 
%     STATE: State structure used by the GA solver 
%     THISSCORE: Vector of scores of the current population 
%     THISPOPULATION: Matrix of individuals in the current population
%     MUTATIONRATE: Rate of mutation

% Here we swap two elements of the permutation
mutationChildren = cell(length(parents),1);% Normally zeros(length(parents),NVARS);
for i=1:length(parents)
    parent = thisPopulation{parents(i)}; % Normally thisPopulation(parents(i),:)
    p = ceil(length(parent) * rand(1,2));
    child = parent;
    child(p(1)) = parent(p(2));
    child(p(2)) = parent(p(1));
    mutationChildren{i} = child; % Normally mutationChildren(i,:)
end

end

function scores=fitness_function(x,cand,distance_segments_ref,alldist_markers)
end

function cand=Predict_fromEllipsoid(time,markers_in_columns,imark)
    global frame;
    
    %fit ellipse
    elips=fit_ellipse(markers_in_columns(1:frame-1,imark*2-1),markers_in_columns(1:frame-1,imark*2));
    ParG=[elips.X0_in elips.Y0_in elips.long_axis/2 elips.short_axis/2 elips.phi];
    
    %predict point with 
    x=markers_in_columns(1:frame-1,imark*2-1);
    y=markers_in_columns(1:frame-1,imark*2);
    theta_raw=atan2(y-elips.Y0_in,x-elips.X0_in);
    if median(diff(theta_raw))<0
        leg='L';
        pos=find(diff(theta_raw)>0);
        pos(end+1)=length(theta_raw);
        if ~isempty(pos)>0;
            theta=theta_raw(1:pos(1));
            for i=2:length(pos)
               theta(pos(i-1)+1:pos(i))=theta_raw(pos(i-1)+1:pos(i))-2*pi*(i-1); 
            end
        end
    else
        leg='R';
        pos=find(diff(theta_raw)<0);
        pos(end+1)=length(theta_raw);
        if ~isempty(pos)>0;
            theta=theta_raw(1:pos(1));
            for i=2:length(pos)
               theta(pos(i-1)+1:pos(i))=theta_raw(pos(i-1)+1:pos(i))+2*pi*(i-1); 
            end
        end
    end
    
    
    try
        if size(theta,2)>size(theta,1)
            ftheta=fit(time(1:frame-1)',theta','smoothingspline');
        else
            ftheta=fit(time(1:frame-1)',theta,'smoothingspline');
        end
    catch
        keyboard;
    end
    theta_fit=ftheta(time(frame));
    x_fit=elips.X0_in+elips.a*cos(theta_fit);
    y_fit=elips.Y0_in+elips.b*sin(theta_fit);
    
    [RSS, XYproj] = Residuals_ellipse([x_fit y_fit],ParG);
    cand=XYproj;
end
function stats_point=FindIntersec(stats_5points,distance_segments_ref,nm);
    switch nm
        case 1
            keyboard;
        case 2
            keyboard;
        case 3
            P1=stats_5points{end}(4).Centroid;
            P2=stats_5points{end}(5).Centroid;
            l1=distance_segments_ref(3);
            l2=distance_segments_ref(4);
            syms x y            
            sol=solve((x-P1(1))^2+(y-P1(2))^2-l1^2,(x-P2(1))^2+(y-P2(2))^2-l2^2);
        case 4 
            P1=stats_5points{end}(3).Centroid;
            P2=stats_5points{end}(5).Centroid;
            l1=distance_segments_ref(3);
            l2=distance_segments_ref(5);
            syms x y            
            sol=solve((x-P1(1))^2+(y-P1(2))^2-l1^2,(x-P2(1))^2+(y-P2(2))^2-l2^2);
        case 5
            P1=stats_5points{end}(3).Centroid;
            P2=stats_5points{end}(4).Centroid;
            l1=distance_segments_ref(4);
            l2=distance_segments_ref(5);
            syms x y            
            sol=solve((x-P1(1))^2+(y-P1(2))^2-l1^2,(x-P2(1))^2+(y-P2(2))^2-l2^2);
            
    end
cand(1,:)=eval([sol.x(1) sol.y(1)]);
cand(2,:)=eval([sol.x(2) sol.y(2)]);
[m I]=min([norm(cand(1,:)-stats_5points{end-1}(nm).Centroid) norm(cand(2,:)-stats_5points{end-1}(nm).Centroid)]);
stats_point.Centroid=cand(I,:);
end
function foot_angle=calc_foot_angle(stats_5points,leg);
    front=stats_5points(4).Centroid;
    heel=stats_5points(5).Centroid;
    if exist('leg')
        if strcmp(leg,'right')
            foot_angle=atan2(front(2)-heel(2),front(1)-heel(1));
        elseif strcmp(leg,'left');
            foot_angle=atan2(front(2)-heel(2),-front(1)+heel(1));
        end
    else
        if front(1)>heel(1)
            foot_angle=atan2(front(2)-heel(2),front(1)-heel(1));
        elseif front(1)<=heel(1)
            foot_angle=atan2(front(2)-heel(2),-front(1)+heel(1));
        end
    end

end
function foot_angle_err=Calc_foot_angle_err(t,foot_angle,cand_heel,cand_front);
global frame;
x=t(1:frame-1)';
y=foot_angle';

fy=fit(x,y,'smoothingspline');
y_fit=fy(t(frame));

for i=1:size(cand_front,1)
    for j=1:size(cand_heel,1)
        stats(4).Centroid=cand_front(i,:);
        stats(5).Centroid=cand_heel(j,:);
        cand_foot_angle=calc_foot_angle(stats);
        foot_angle_err(i,j)=(y_fit-cand_foot_angle)*180/pi;
    end
end
end
function [front heel]=Pred_front_heel_using_footangle(t,foot_angle,stats_5points,distance_segments_ref,leg);
    global frame;
    x0=t(1:frame-1);
    y0=foot_angle;
    fy=fit(x0',y0','smoothingspline');
    y_fit=fy(t(frame));
    
    ankle=stats_5points(3).Centroid;
    try
        F=@(x)fun_sol(x,leg,y_fit,distance_segments_ref,ankle);
        if strcmp(leg,'right')
            x_init(1)=ankle(1)+distance_segments_ref(3);
            x_init(2)=ankle(2)+distance_segments_ref(3);
            x_init(3)=ankle(1)-distance_segments_ref(4);
            x_init(4)=ankle(2)+distance_segments_ref(4);
        elseif strcmp(leg,'left')
            x_init(1)=ankle(1)-distance_segments_ref(3);
            x_init(2)=ankle(2)+distance_segments_ref(3);
            x_init(3)=ankle(1)+distance_segments_ref(4);
            x_init(4)=ankle(2)+distance_segments_ref(4);
        end
        sol=fsolve(F,x_init);
        
%         sol=solve(calc_foot_angle(stats,leg)-y_fit,...
%             norm([x_front y_front]-[x_heel y_heel])-distance_segments_ref(5),...
%             norm(ankle-[x_heel y_heel])-distance_segments_ref(4),...
%             norm(ankle-[x_front y_front])-distance_segments_ref(3));
    catch
        keyboard;
    end
    front=[sol(1) sol(2)];
    heel=[sol(3) sol(4)];
end

function [front, heel]=Pred_front_heel_using_footangle_fromEllipsoid(time,foot_angle,stats_5points,distance_segments_ref,leg,abs_angles);

    global frame;
    femur_angle=extractfield(abs_angles,'femur');
    x0=femur_angle(1:frame-1);
    y0=foot_angle;
    
    if frame>20
        try
            elips=fit_ellipse(x0(1:frame-2),y0(1:frame-2));
        catch
            keyboard;
        end
        ParG=[elips.X0_in elips.Y0_in elips.long_axis/2 elips.short_axis/2 elips.phi];
        
        
        theta_raw=atan2(y0-elips.Y0_in,x0-elips.X0_in);
        if median(diff(theta_raw))<0
%             leg='left';
            pos=find(diff(theta_raw)>0);
            pos(frame+1)=length(theta_raw);
            if ~isempty(pos)>0;
                theta=theta_raw(1:pos(1));
                for i=2:length(pos)
                   theta(pos(i-1)+1:pos(i))=theta_raw(pos(i-1)+1:pos(i))-2*pi*(i-1); 
                end
            end
        else
%             leg='R';
            pos=find(diff(theta_raw)<0);
            pos(frame+1)=length(theta_raw);
            if ~isempty(pos)>0;
                theta=theta_raw(1:pos(1));
                for i=2:length(pos)
                   theta(pos(i-1)+1:pos(i))=theta_raw(pos(i-1)+1:pos(i))+2*pi*(i-1); 
                end
            end
        end

        try
            if size(theta,2)>size(theta,1)
                ftheta=fit(time(1:frame-1)',theta','smoothingspline');
            else
                ftheta=fit(time(1:frame-1)',theta,'smoothingspline');
            end
        catch
            keyboard;
        end
        theta_fit=ftheta(time(frame));
        x_fit=elips.X0_in+elips.a*cos(theta_fit);
        y_fit=elips.Y0_in+elips.b*sin(theta_fit);

        [RSS, XYproj] = Residuals_ellipse([x_fit y_fit],ParG);
 
        try
            y_fit=XYproj(2);
        catch
            keyboard;
        end
        
    else
        fy=fit(time(1:frame-1)',y0','smoothingspline');
        y_fit=fy(time(frame));
    end
    


    ankle=stats_5points(3).Centroid;
    try
        F=@(x)fun_sol(x,leg,y_fit,distance_segments_ref,ankle);
        if strcmp(leg,'right')
            x_init(1)=ankle(1)+distance_segments_ref(3);
            x_init(2)=ankle(2)+distance_segments_ref(3);
            x_init(3)=ankle(1)-distance_segments_ref(4);
            x_init(4)=ankle(2)+distance_segments_ref(4);
        elseif strcmp(leg,'left')
            x_init(1)=ankle(1)-distance_segments_ref(3);
            x_init(2)=ankle(2)+distance_segments_ref(3);
            x_init(3)=ankle(1)+distance_segments_ref(4);
            x_init(4)=ankle(2)+distance_segments_ref(4);
        end
        sol=fsolve(F,x_init);
        
%         sol=solve(calc_foot_angle(stats,leg)-y_fit,...
%             norm([x_front y_front]-[x_heel y_heel])-distance_segments_ref(5),...
%             norm(ankle-[x_heel y_heel])-distance_segments_ref(4),...
%             norm(ankle-[x_front y_front])-distance_segments_ref(3));
    catch
        keyboard;
    end
    front=[sol(1) sol(2)];
    heel=[sol(3) sol(4)];
end

function F=fun_sol(x,leg,y_fit,distance_segments_ref,ankle);
%x(1) x_front
%x(2) y_front
%x(3) x_heel
%x(4) y_heel
stats(4).Centroid=[x(1) x(2)];
stats(5).Centroid=[x(3) x(4)];
F=[calc_foot_angle(stats,leg)-y_fit,...
    norm([x(1) x(2)]-[x(3) x(4)])-distance_segments_ref(5),...
    norm(ankle-[x(3) x(4)])-distance_segments_ref(4),...
    norm(ankle-[x(1) x(2)])-distance_segments_ref(3)];
end
function cand=calculate_missing_point_OpenP(stats_5points,t,foot_angle,dist_segments,n_m)

global frame;
x=t(1:frame-1)';
y=foot_angle';

fy=fit(x,y,'smoothingspline');
y_fit=fy(t(frame));

if n_m==4;
   cand(1,:)=stats_5points(5).Centroid+dist_segments(5)*[-cos(y_fit) -sin(y_fit)];
   syms x y;
   sol=solve(sum(([x y]-stats_5points(5).Centroid).^2)-dist_segments(5)^2,...
       sum(([x y]-stats_5points(3).Centroid).^2)-dist_segments(3)^2);
   cand(2,:)=eval([sol.x(1) sol.y(1)]);
   cand(3,:)=eval([sol.x(2) sol.y(2)]);
elseif n_m==5;
   cand(1,:)=stats_5points(4).Centroid+dist_segments(5)*[cos(y_fit) sin(y_fit)];
   syms x y;
   sol=solve(sum(([x y]-stats_5points(4).Centroid).^2)-dist_segments(5)^2,...
       sum(([x y]-stats_5points(3).Centroid).^2)-dist_segments(4)^2);
   cand(2,:)=eval([sol.x(1) sol.y(1)]);
   cand(3,:)=eval([sol.x(2) sol.y(2)]);
end


end

function [cand_front cand_heel]=calculate_missing_heel_front_OpenP(stats_5points,t,foot_angle,distance_segments_ref);
global frame;
cand_ankle=stats_5points{frame}(3);

x=t(1:frame-1)';
y=foot_angle';

fy=fit(x,y,'smoothingspline');
y_fit=fy(t(frame));

syms x_heel y_heel x_front y_front;
if stats_5points{frame-1}(4).Centroid(1)>stats_5points{frame-1}(5).Centroid(1)
    eq4=atan2(y_front-y_heel,x_front-x_heel)-y_fit;
else
    eq4=atan2(y_front-y_heel,-x_front+x_heel)-y_fit;
end
sol=solve(...
    sum(([x_heel y_heel]-cand_ankle.Centroid).^2)-distance_segments_ref(4)^2,...
    sum(([x_front y_front]-cand_ankle.Centroid).^2)-distance_segments_ref(3)^2,...
    sum(([x_front y_front]-[x_heel y_heel]).^2)-distance_segments_ref(5)^2,...
    eq4, [x_heel y_heel x_front y_front]);

cand_front=eval([sol.x_front sol.y_front]);
cand_heel=eval([sol.x_heel sol.y_heel]);

end

function  [stats_5points, foot_angle, abs_angles]=CheckFinalPointsFrame(stats_5points,foot_angle,distance_segments_ref,time,angles_vectors_ref,leg,abs_angles,markers_in_columns,video_data);
global error_log frame;

hip=stats_5points{frame}(1).Centroid;
knee=stats_5points{frame}(2).Centroid;
ankle=stats_5points{frame}(3).Centroid;
front=stats_5points{frame}(4).Centroid;
heel=stats_5points{frame}(5).Centroid;

dist_femur=sqrt(sum((hip-knee).^2));
dist_tibia=sqrt(sum((knee-ankle).^2));
dist_ankletoe=sqrt(sum((ankle-front).^2));
dist_ankleheel=sqrt(sum((ankle-heel).^2));
dist_heeltoe=sqrt(sum((heel-front).^2));

if frame>20
    femur_angle=extractfield(abs_angles,'femur');
    fit_femur_angle=fit(time(1:frame-1)',femur_angle(1:frame-1)','sin2');
    if rms(fit_femur_angle(time(1:frame-1))-femur_angle(1:frame-1)')<0.1
    else
        fit_femur_angle=fit(time(1:frame-1)',femur_angle(1:frame-1)','sin3');
        if rms(fit_femur_angle(time(1:frame-1))-femur_angle(1:frame-1)')<0.1
        else
            fit_femur_angle=fit(time(1:frame-1)',femur_angle(1:frame-1)','sin4');
        end
    end
    femur_angle_pred=fit_femur_angle(time(frame));
    if femur_angle_pred<min(femur_angle)
        femur_angle_pred=min(femur_angle);
    elseif femur_angle_pred>max(femur_angle)
        femur_angle_pred=max(femur_angle);
    end
 
    tibia_angle=extractfield(abs_angles,'tibia');
    fit_tibia_angle=fit(time(1:frame-1)',tibia_angle(1:frame-1)','sin2');
    if rms(fit_tibia_angle(time(1:frame-1))-tibia_angle(1:frame-1)')<0.1
    else
        fit_tibia_angle=fit(time(1:frame-1)',tibia_angle(1:frame-1)','sin3');
        if rms(fit_tibia_angle(time(1:frame-1))-tibia_angle(1:frame-1)')<0.1
        else
            fit_tibia_angle=fit(time(1:frame-1)',tibia_angle(1:frame-1)','sin4');
        end
    end
    tibia_angle_pred=fit_tibia_angle(time(frame));
    if tibia_angle_pred<min(tibia_angle)
        tibia_angle_pred=min(tibia_angle);
    elseif tibia_angle_pred>max(tibia_angle)
        tibia_angle_pred=max(tibia_angle);
    end
    
    if (norm(stats_5points{frame}(1).Centroid-stats_5points{frame-1}(1).Centroid)>150)
%         if length(foot_angle)>40
            hip=stats_5points{frame-1}(1).Centroid
            femur_angle(frame)=femur_angle_pred;
            knee=hip-distance_segments_ref(1)*[cos(femur_angle(frame)) sin(femur_angle(frame))];
            tibia_angle(frame)=tibia_angle_pred;
            ankle=knee-distance_segments_ref(2)*[cos(tibia_angle(frame)) sin(tibia_angle(frame))];
            [front, heel]=Pred_front_heel_using_footangle_fromEllipsoid(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg,abs_angles);
            stats_5points{end}(1).Centroid=hip;
            stats_5points{end}(2).Centroid=knee;
            stats_5points{end}(3).Centroid=ankle;
            stats_5points{end}(4).Centroid=front;
            stats_5points{end}(5).Centroid=heel;
%         else
%             keyboard;
%         end
        error_log(frame)=4;
    elseif abs(femur_angle_pred-femur_angle(frame))>15*pi/180
        hip=stats_5points{frame}(1).Centroid;
        femur_angle(frame)=femur_angle_pred;
        knee=hip-distance_segments_ref(1)*[cos(femur_angle(frame)) sin(femur_angle(frame))];
        tibia_angle(frame)=tibia_angle_pred;
        ankle=knee-distance_segments_ref(2)*[cos(tibia_angle(frame)) sin(tibia_angle(frame))];
        [front, heel]=Pred_front_heel_using_footangle_fromEllipsoid(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg,abs_angles);
        stats_5points{frame}(2).Centroid=knee;
        stats_5points{frame}(3).Centroid=ankle;
        stats_5points{frame}(4).Centroid=front;
        stats_5points{frame}(5).Centroid=heel;
        error_log(frame)=5;
    elseif abs(tibia_angle_pred-tibia_angle(frame))>20*pi/180
        knee=stats_5points{frame}(2).Centroid;
        tibia_angle(frame)=tibia_angle_pred;
%         ankle=knee-distance_segments_ref(2)*[cos(tibia_angle(frame)) sin(tibia_angle(frame))];
        ankle=Predict_fromEllipsoid(time,markers_in_columns,3);
        stats_5points{frame}(3).Centroid=ankle;
        [front, heel]=Pred_front_heel_using_footangle_fromEllipsoid(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg,abs_angles);
        
        stats_5points{frame}(4).Centroid=front;
        stats_5points{frame}(5).Centroid=heel;
        error_log(frame)=6;
    end
else
    if ((norm(stats_5points{frame}(1).Centroid-stats_5points{frame-1}(1).Centroid)>150)|...
            (norm(stats_5points{frame}(2).Centroid-stats_5points{frame-1}(2).Centroid)>150)|...
            (norm(stats_5points{frame}(3).Centroid-stats_5points{frame-1}(3).Centroid)>150))
        [stats_5points{frame}]=PickFirstPoint_ind_OpenPose_manual(video_data);
        femur_angle=extractfield(abs_angles,'femur');
        tibia_angle=extractfield(abs_angles,'tibia');
    end
end

change=0;
if abs((dist_femur-distance_segments_ref(1)))/distance_segments_ref(1)>0.2
    knee=hip-distance_segments_ref(1)*[cos(femur_angle(frame)) sin(femur_angle(frame))];
    ankle=knee-distance_segments_ref(2)*[cos(tibia_angle(frame)) sin(tibia_angle(frame))];
    stats_5points{frame}(2).Centroid=knee;
    stats_5points{frame}(3).Centroid=ankle;
    if length(foot_angle)<20
        [front, heel]=Pred_front_heel_using_footangle(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg);
        error_log(frame)=7;
    else
        [front, heel]=Pred_front_heel_using_footangle_fromEllipsoid(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg,abs_angles);
        error_log(frame)=7.5;
    end    
    stats_5points{frame}(4).Centroid=front;
    stats_5points{frame}(5).Centroid=heel;
    if frame>20
        [pks,locs,widths,proms]=findpeaks(femur_angle,time,'MinPeakDistance',0.4);
        if length(pks)>10
            stats_5points=stats_5points{frame};
            abs_angles=calc_abs_angles(stats_5points);
            return;
        else
        end
    end
    change=1;
elseif abs((dist_tibia-distance_segments_ref(2)))/distance_segments_ref(2)>0.2
    if frame>20
        [pks,locs,widths,proms]=findpeaks(tibia_angle,time,'MinPeakDistance',0.4);
        if length(pks>10)
            stats_5points=stats_5points{frame};
            abs_angles=calc_abs_angles(stats_5points);
            return;
        else
        end
    end
    change=1;
elseif abs((dist_ankletoe-distance_segments_ref(3)))/distance_segments_ref(3)>0.4
    error_ankleheel=abs((dist_ankleheel-distance_segments_ref(4)))/distance_segments_ref(4)>0.2;
    error_frontheel=abs((dist_heeltoe-distance_segments_ref(5)))/distance_segments_ref(5)>0.2;
    if error_ankleheel&error_frontheel
        [front, heel]=Pred_front_heel_using_footangle(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg);
        error_log(frame)=8;
    end
    stats_5points{frame}(4).Centroid=full(front);
    stats_5points{frame}(5).Centroid=full(heel);
    change=1;
elseif abs((dist_ankleheel-distance_segments_ref(4)))/distance_segments_ref(4)>0.4
    error_frontheel=abs((dist_heeltoe-distance_segments_ref(5)))/distance_segments_ref(5)>0.25;
    if error_frontheel
        stats_5points_aux{1}=stats_5points;
        foot_angle_aux=foot_angle(1:frame-1);
        [front heel]=Pred_front_heel_using_footangle(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg);
        error_log(frame)=9;
    end
    stats_5points{frame}(4).Centroid=full(front);
    stats_5points{frame}(5).Centroid=full(heel);
    change=1;    
elseif abs((dist_heeltoe-distance_segments_ref(5)))/distance_segments_ref(5)>0.4
    keyboard;
end

if change
    foot_angle(frame)=calc_foot_angle(stats_5points{frame});
end

change=0;
if abs(abs_angles(frame).femur-abs_angles(frame-1).femur)>(30*pi/180)
    keyboard;
elseif abs(abs_angles(frame).tibia-abs_angles(frame-1).tibia)>(30*pi/180)
    if length(abs_angles)>20
        ankle=Predict_fromEllipsoid(time,markers_in_columns,3);
        error_log(frame)=10;
    else
        ankle=Predict_fromPoly(time,markers_in_columns,3);
        error_log(frame)=10.5;
    end
    stats_5points{frame}(3).Centroid=ankle;
    [front heel]=Pred_front_heel_using_footangle(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg);
    stats_5points{frame}(3).Centroid=full(ankle);
    stats_5points{frame}(4).Centroid=full(front);
    stats_5points{frame}(5).Centroid=full(heel);
    change=1;
end

if change
    foot_angle(frame)=calc_foot_angle(stats_5points{frame});
end

change=0;
if abs(foot_angle(frame))>(50*pi/180)
    if length(foot_angle)<20
        [front, heel]=Pred_front_heel_using_footangle(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg);
        error_log(frame)=11;
    else
        [front, heel]=Pred_front_heel_using_footangle_fromEllipsoid(time,foot_angle(1:frame-1),stats_5points{frame},distance_segments_ref,leg,abs_angles);
        error_log(frame)=11.5;
    end
    stats_5points{frame}(4).Centroid=full(front);
    stats_5points{frame}(5).Centroid=full(heel);
    change=1;    
end

if change
    foot_angle(frame)=calc_foot_angle(stats_5points{frame});
end
stats_5points=stats_5points{frame};

abs_angles=calc_abs_angles(stats_5points);

end

function     abs_angles=calc_abs_angles(stats_5points);
hip=stats_5points(1).Centroid;
knee=stats_5points(2).Centroid;
ankle=stats_5points(3).Centroid;
front=stats_5points(4).Centroid;
heel=stats_5points(5).Centroid;

abs_angles.femur=atan2(hip(2)-knee(2),hip(1)-knee(1));
abs_angles.tibia=atan2(knee(2)-ankle(2),knee(1)-ankle(1));
abs_angles.ankletoe=atan2(ankle(2)-front(2),ankle(1)-front(1));
abs_angles.ankleheel=atan2(ankle(2)-heel(2),ankle(1)-heel(1));
abs_angles.heeltoe=atan2(heel(2)-front(2),heel(1)-front(1));

end
function cand=Predict_fromPoly(time,markers_in_columns,imark);

    fun_x=fit(time(1:end-1)',markers_in_columns(:,imark*2-1),'smoothingspline');
    fun_y=fit(time(1:end-1)',markers_in_columns(:,imark*2),'smoothingspline');
    cand(1)=fun_x(time(end));
    cand(2)=fun_y(time(end));

end