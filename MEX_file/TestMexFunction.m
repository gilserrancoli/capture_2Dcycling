%% Author: Gil Serrancolí
%Read the image example and extracts the keypoints of all skeletons
%detected at the image. Note that the file Process_Image_MEX should be in
%your path.
inputImage=double(imread('upc_running.jpg'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Funció MEX
[a b c pose(:,:,1) pose(:,:,2) pose(:,:,3)]=Process_Image_MEX(inputImage);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot solution
Colors(1,:)=[0 0 1];
Colors(2,:)=[0 1 0];
Colors(3,:)=[0 1 1];
Colors(4,:)=[1 0 0];
Colors(5,:)=[1 0 1];
Colors(6,:)=[1 1 0];
Colors(7,:)=[1 1 1];
Colors(8,:)=[0.5 0.5 0.5];
imshow(imread('upc_running.jpg'));
hold all;
for i=1:size(pose,1)
   plot(pose(i,:,1),pose(i,:,2),'o','LineWidth',2,'Color',Colors(i,:)); 
   hold all;
end
% set(gca,'Xdir','reverse','Ydir','reverse');
    