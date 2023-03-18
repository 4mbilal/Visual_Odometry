% clear all
% close all
% clc

rng(0);

% load("gyro.mat");
% load("yaw2d.mat");
% load("yaw3d.mat");
% load("yawquality.mat");


% load("gyro_KITE_0003.mat");
% load("yaw2d_KITE_0003.mat");
% load("yaw3d_KITE_0003.mat");
% load("yawquality_KITE_0003.mat");

% load("gyro_KITE_0008.mat");
% load("yaw2d_KITE_0008.mat");
% load("yaw3d_KITE_0008.mat");
% load("yawquality_KITE_0008.mat");

% yaw_2D(yaw_2D<-80) = -80;

gyr_bias = 5;
gyr_noise = randn(size(gyro_gt))*10;
gyro_noisy = gyro_gt + gyr_noise + gyr_bias;

a = 0.95;
a = min(1,yaw_2D_q/100);
% 4.2698 400
fuse_2D = yaw_2D.*a + gyro_noisy.*(1-a);

% yaw_3D = yaw_2D + randn(size(yaw_2D))*10;
fuse_3D = yaw_3D.*a + gyro_noisy.*(1-a);

yaw_gt = gyro_gt;
yaw_noisy = gyro_noisy;
yaw_fused_2D = fuse_2D;
yaw_fused_3D = fuse_3D;

for i=2:length(fuse_2D)
    yaw_gt(i) = (yaw_gt(i) + yaw_gt(i-1));
    yaw_noisy(i) = (yaw_noisy(i) + yaw_noisy(i-1));
    yaw_fused_2D(i) = (yaw_fused_2D(i) + yaw_fused_2D(i-1));
    yaw_fused_3D(i) = (yaw_fused_3D(i) + yaw_fused_3D(i-1));
end


figure
plot(yaw_gt* 0.0083)
hold on
plot(yaw_noisy* 0.0083,'-.','LineWidth',2)
% plot(yaw_fused_3D* 0.0083,'.','LineWidth',2)
plot(yaw_fused_2D* 0.0083,'--','LineWidth',2)
% plot(fuse_2D)
xlabel('Sample No.')
ylabel('Yaw (radians)')
% legend('Ground Truth','Noisy + Biased Measurement','Fused Measurement with 3D visual pose', 'Fused Measurement with 2D visual pose (Proposed)')
legend('Ground Truth','Noisy + Biased Measurement', 'Fused Measurement with 2D visual pose (Proposed)')
set(gca,'FontSize',20)
mse_2D = sum(abs(fuse_2D-gyro_gt).^2)/length(fuse_2D)
mse_3D = sum(abs(fuse_3D-gyro_gt).^2)/length(fuse_2D)
mse_noisy = sum(abs(gyro_noisy-gyro_gt).^2)/length(fuse_2D)
% 
% mse_2D = sum(abs(yaw_fused_2D-yaw_gt).^2)/length(fuse_2D)
% mse_3D = sum(abs(yaw_fused_3D-yaw_gt).^2)/length(fuse_2D)
% mse_noisy = sum(abs(yaw_noisy-yaw_gt).^2)/length(fuse_2D)


