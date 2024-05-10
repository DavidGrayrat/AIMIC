close all;clear;

shiftrange=1;
f_ecd=1000;
dt_ecd=1/f_ecd;
f_w_joint_min=pi/(4*shiftrange);
totaltime=2*pi/f_w_joint_min;
t_ecd=0:dt_ecd:totaltime;
K_num=10;
K_f_w_joint=(1:K_num);
joint=t_ecd.'*f_w_joint_min*K_f_w_joint;
num_joint=3;

K1=0.1*ones(K_num,1);
K2=0.1*ones(K_num,1);
K3=zeros(K_num,1);
w_joint1=sin(joint)*K1;
w_joint2=cos(joint)*K2;
w_joint3=cos(joint)*K3;
w_joint=[w_joint1,w_joint2,w_joint3];
theta_joint=zeros(length(t_ecd),num_joint);
theta_joint(1,:)=[0,0,-1.77];
for i=2:length(t_ecd)
    theta_joint(i,:)=theta_joint(i-1,:)+w_joint(i-1,:)*dt_ecd;
end
w_imu_i=[-w_joint1.*sin(theta_joint(:,2)+theta_joint(:,3)),-w_joint1.*cos(theta_joint(:,2)+theta_joint(:,3)),w_joint2+w_joint3];
wx=w_imu_i(:,1)-mean(w_imu_i(:,1));
wy=w_imu_i(:,2)-mean(w_imu_i(:,2));
wz=w_imu_i(:,3)-mean(w_imu_i(:,3));
sigma_ii=[mean(wx.*wx),mean(wx.*wy),mean(wx.*wz);
          mean(wy.*wx),mean(wy.*wy),mean(wy.*wz);
          mean(wz.*wx),mean(wz.*wy),mean(wz.*wz)];
condition_num=cond(sigma_ii);
theta1range=max(theta_joint(:,1))-min(theta_joint(:,1));
theta2range=max(theta_joint(:,2))-min(theta_joint(:,2));

alpha = 1e-3;  
max_iter = 10000;  
loss_function = @(K1, K2) compute_condition_num(K1, K2);  
for iter = 1:max_iter
    [grad_K1, grad_K2] = gradient(loss_function, K1, K2);
    K1 = K1 - alpha * grad_K1;
    K2 = K2 - alpha * grad_K2;
    [new_cond_num,loss_iter] = loss_function(K1, K2);
    disp([iter,new_cond_num,loss_iter])
    if new_cond_num < 1.01 && loss_iter <2
        disp('Convergence met');
        break;
    end
    if loss_iter>100
        disp('?')
        break;
    end
end

disp('Optimized K1:');
disp(K1);
disp('Optimized K2:');
disp(K2);
K3=zeros(K_num,1);
w_joint1=sin(joint)*K1;
w_joint2=cos(joint)*K2;
w_joint3=cos(joint)*K3;
w_joint=[w_joint1,w_joint2,w_joint3];
theta_joint=zeros(length(t_ecd),num_joint);
theta_joint(1,:)=[0,0,-1.77];
for i=2:length(t_ecd)
    theta_joint(i,:)=theta_joint(i-1,:)+w_joint(i-1,:)*dt_ecd;
end
w_imu_i=[-w_joint1.*sin(theta_joint(:,2)+theta_joint(:,3)),-w_joint1.*cos(theta_joint(:,2)+theta_joint(:,3)),w_joint2+w_joint3];
wx=w_imu_i(:,1)-mean(w_imu_i(:,1));
wy=w_imu_i(:,2)-mean(w_imu_i(:,2));
wz=w_imu_i(:,3)-mean(w_imu_i(:,3));
sigma_ii=[mean(wx.*wx),mean(wx.*wy),mean(wx.*wz);
          mean(wy.*wx),mean(wy.*wy),mean(wy.*wz);
          mean(wz.*wx),mean(wz.*wy),mean(wz.*wz)];
lambda_max=max(abs(eig(sigma_ii)));
lambda_min=min(abs(eig(sigma_ii)));
cond_num_sigma_gg=lambda_max/lambda_min;
condition_num=cond(sigma_ii);
theta1range=max(theta_joint(:,1))-min(theta_joint(:,1));
theta2range=max(theta_joint(:,2))-min(theta_joint(:,2));
figure;
subplot(2,2,1);plot(t_ecd,theta_joint(:,1));
subplot(2,2,2);plot(t_ecd,theta_joint(:,2));
subplot(2,2,3);plot(t_ecd,w_joint1);
subplot(2,2,4);plot(t_ecd,w_joint2);

function [grad_K1, grad_K2] = gradient(loss_function, K1, K2)
    delta = 1e-6;  
    K_num=length(K1);
    grad_K1=zeros(K_num,1);
    grad_K2=zeros(K_num,1);
    for i=1:K_num
        dlt=zeros(K_num,1);
        dlt(i)=delta;
        [cdn_11,los_11]=loss_function(K1 + dlt, K2);
        [cdn_10,los_10]=loss_function(K1 - dlt, K2);
        grad_K1(i)=(los_11 - los_10) / (2 * delta);
        [cdn_21,los_21]=loss_function(K1, K2 + dlt);
        [cdn_20,los_20]=loss_function(K1, K2 - dlt);
        grad_K2(i)=(los_21 - los_20) / (2 * delta);
    end
end

function [cond_num, loss] = compute_condition_num(K1, K2)
    shiftrange=1;
    f_ecd=1000;
    dt_ecd=1/f_ecd;
    f_w_joint_min=pi/(4*shiftrange);
    totaltime=2*pi/f_w_joint_min;
    t_ecd=0:dt_ecd:totaltime;
    K_num=10;
    K_f_w_joint=(1:K_num);
    joint=t_ecd.'*f_w_joint_min*K_f_w_joint;
    num_joint=3;
    K3=zeros(K_num,1);
    w_joint1=sin(joint)*K1;
    w_joint2=cos(joint)*K2;
    w_joint3=cos(joint)*K3;
    w_joint=[w_joint1,w_joint2,w_joint3];
    theta_joint=zeros(length(t_ecd),num_joint);
    theta_joint(1,:)=[0,0,-1.77];
    for i=2:length(t_ecd)
        theta_joint(i,:)=theta_joint(i-1,:)+w_joint(i-1,:)*dt_ecd;
    end
    w_imu_i=[-w_joint1.*sin(theta_joint(:,2)+theta_joint(:,3)),-w_joint1.*cos(theta_joint(:,2)+theta_joint(:,3)),w_joint2+w_joint3];
    wx=w_imu_i(:,1)-mean(w_imu_i(:,1));
    wy=w_imu_i(:,2)-mean(w_imu_i(:,2));
    wz=w_imu_i(:,3)-mean(w_imu_i(:,3));
    sigma_ii=[mean(wx.*wx),mean(wx.*wy),mean(wx.*wz);
              mean(wy.*wx),mean(wy.*wy),mean(wy.*wz);
              mean(wz.*wx),mean(wz.*wy),mean(wz.*wz)];
    cond_num=cond(sigma_ii);

    theta1range=max(theta_joint(:,1))-min(theta_joint(:,1));
    theta2range=max(theta_joint(:,2))-min(theta_joint(:,2));
    theta1limit=pi/4;
    theta2limit=pi/2;
    weight_theta1=1;
    weight_theta2=1;
    loss=cond_num;
    if theta1range>=theta1limit
        loss = loss+weight_theta1*theta1range;
    end
    if theta2range>=theta2limit
        loss = loss+weight_theta2*theta2range;
    end
end