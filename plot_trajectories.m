clear all;
close all;


dir = '/home/abeauvisage/CityLondon/implementation/StereoLocalisation/rsrc/Ibis2/poses';

tracking = load([dir '/poses_tracking.csv']);
slam = load([dir '/poses_slam.csv']);
viso = load([dir '/poses_viso.csv']);

gt_position = tracking(:,8:10);
tracking_position = tracking(:,2:4);
slam_position = slam(:,2:4);
viso_position = viso(:,2:4);
gt_orientation = tracking(:,11:13);
tracking_orientation = tracking(:,5:7);
slam_orientation = slam(:,5:7);
viso_orientation = viso(:,5:7);

errp_tracking = (gt_position - tracking_position).^2;
errp_viso = (gt_position - viso_position).^2;
errp_slam = (gt_position - slam_position).^2;
erro_tracking = (gt_orientation - tracking_orientation).^2;
erro_viso = (gt_orientation - viso_orientation).^2;
erro_slam = (gt_orientation - slam_orientation).^2;

AEE = [  mean(sum(errp_tracking,2).^0.5) mean(sum(erro_tracking,2).^0.5);
        mean(sum(errp_viso,2).^0.5) mean(sum(erro_viso,2).^0.5);
        mean(sum(errp_slam,2).^0.5) mean(sum(erro_slam,2).^0.5)]

MSE = [ mean(errp_tracking(:,1)) mean(errp_tracking(:,2)) mean(errp_tracking(:,3)) mean(erro_tracking(:,1)) mean(erro_tracking(:,2)) mean(erro_tracking(:,3));
        mean(errp_viso(:,1)) mean(errp_viso(:,2)) mean(errp_viso(:,3)) mean(erro_viso(:,1)) mean(erro_viso(:,2)) mean(erro_viso(:,3));
        mean(errp_slam(:,1)) mean(errp_slam(:,2)) mean(errp_slam(:,3)) mean(erro_slam(:,1)) mean(erro_slam(:,2)) mean(erro_slam(:,3))]
        
RMSE = MSE.^(0.5)


figure(1)
  title1 = title('Stereo localisation: position');
  set(title1, "FontSize",16);
  xlabel('X (m)')
  ylabel('Y (m)')
  palette = get(gca,'colororder');
  hold on
  plot(gt_position(:,2),gt_position(:,1),'color',palette(1,:))
  plot(viso_position(:,2),viso_position(:,1),'color',palette(2,:))
  plot(tracking_position(:,2),tracking_position(:,1),'color',palette(3,:))
  plot(slam_position(:,2),slam_position(:,1),'color',palette(4,:))

  hold off
  legend1 = legend({'GT','libviso2','tracking','StereoSlam'});
  set(legend1, "FontSize",10);
  set([gca;findall(gca,'Type','text')],"fontsize",14);
  axis("equal")
  filename = [dir '/positionXY.eps'];
  print(filename,"-depsc")
  
figure(2)
  title2 = title('Stereo localisation: position');
  set(title2, "FontSize",16);
  xlabel('X (m)')
  ylabel('Z (m)')
  palette = get(gca,'colororder');
  hold on
  plot(gt_position(:,1),gt_position(:,3),'color',palette(1,:))
  plot(viso_position(:,1),viso_position(:,3),'color',palette(2,:))
  plot(tracking_position(:,1),tracking_position(:,3),'color',palette(3,:))
  plot(slam_position(:,1),slam_position(:,3),'color',palette(4,:))

  hold off
  legend2 = legend({'GT','libviso2','tracking','StereoSlam'});
  set(legend2, "FontSize",10);
  set([gca;findall(gca,'Type','text')],"fontsize",14);
  axis("equal")
  filename = [dir '/positionXZ.eps'];
  print(filename,"-depsc")


figure(3)
  title3 = title('Stereo localisation: position');
  set(title3, "FontSize",16);
  xlabel('Y (m)')
  ylabel('Z (m)')
  palette = get(gca,'colororder');
  hold on
  plot(gt_position(:,2),gt_position(:,3),'color',palette(1,:))
  plot(viso_position(:,2),viso_position(:,3),'color',palette(2,:))
  plot(tracking_position(:,2),tracking_position(:,3),'color',palette(3,:))
  plot(slam_position(:,2),slam_position(:,3),'color',palette(4,:))

  hold off
  legend3 = legend({'GT','libviso2','tracking','StereoSlam'});
  set(legend3, "FontSize",10);
  set([gca;findall(gca,'Type','text')],"fontsize",14);
  axis("equal")
  filename = [dir '/positionYZ.eps'];
  print(filename,"-depsc")
  
 figure(4)
  title4 = title('Stereo localisation: attitude');
  set(title4, "FontSize",16);
  xlabel('X (rad)')
  ylabel('Y (rad)')
  palette = get(gca,'colororder');
  hold on
  plot(1:length(viso(:,1)),gt_orientation(:,1),'color',palette(1,:))
  plot(1:length(viso(:,1)),viso_orientation(:,1),'color',palette(2,:))
  plot(1:length(viso(:,1)),tracking_orientation(:,1),'color',palette(3,:))
  plot(1:length(viso(:,1)),slam_orientation(:,1),'color',palette(4,:))

  hold off
  legend4 = legend({'GT','libviso2','tracking','StereoSlam'},'Location','northwest');
  set(legend4, "FontSize",10);
  set([gca;findall(gca,'Type','text')],"fontsize",14);
  filename = [dir '/orientationXY.eps'];
  print(filename,"-depsc")
  
figure(5)
  title5 = title('Stereo localisation: attitude');
  set(title5, "FontSize",16);
  xlabel('X (rad)')
  ylabel('Z (rad)')
  palette = get(gca,'colororder');
  hold on
  plot(1:length(viso(:,1)),gt_orientation(:,2),'color',palette(1,:))
  plot(1:length(viso(:,1)),viso_orientation(:,2),'color',palette(2,:))
  plot(1:length(viso(:,1)),tracking_orientation(:,2),'color',palette(3,:))
  plot(1:length(viso(:,1)),slam_orientation(:,2),'color',palette(4,:))

  hold off
  legend5 = legend({'GT','libviso2','tracking','StereoSlam'},'Location','northwest');
  set(legend5, "FontSize",10);
  set([gca;findall(gca,'Type','text')],"fontsize",14);
  filename = [dir '/orientationXZ.eps'];
  print(filename,"-depsc")


figure(6)
  title6 = title('Stereo localisation: attitude');
  set(title6, "FontSize",16);
  xlabel('Y (rad)')
  ylabel('Z (rad)')
  palette = get(gca,'colororder');
  hold on
  plot(1:length(viso(:,1)),gt_orientation(:,3),'color',palette(1,:))
  plot(1:length(viso(:,1)),viso_orientation(:,3),'color',palette(2,:))
  plot(1:length(viso(:,1)),tracking_orientation(:,3),'color',palette(3,:))
  plot(1:length(viso(:,1)),slam_orientation(:,3),'color',palette(4,:))

  hold off
  legend6 = legend({'GT','libviso2','tracking','StereoSlam'},'Location','northwest');
  set(legend6, "FontSize",10);
  set([gca;findall(gca,'Type','text')],"fontsize",14);
  filename = [dir '/orientationYZ.eps'];
  print(filename,"-depsc")
  
  figure(9)
  hold on
  palette = get(gca,'colororder');
  plot3(gt_position(:,1),gt_position(:,2),gt_position(:,3),'color',palette(1,:))
  plot3(gt_position(:,1),viso_position(:,2),viso_position(:,3),'color',palette(2,:))
  plot3(gt_position(:,1),tracking_position(:,2),tracking_position(:,3),'color',palette(3,:))
  plot3(gt_position(:,1),slam_position(:,2),slam_position(:,3),'color',palette(4,:))
  hold off