close all;clear;

totaltime=20;
f_ecd=100;
dt_ecd=1/f_ecd;
shiftrange=1;
t_ecd=0:dt_ecd:totaltime;

num_joint=4;
theta_joint=zeros(length(t_ecd),num_joint);
theta_joint(1,:)=[0,0,-1.78,0];
theta_joint_floor=[-pi/6,pi/3,0,-pi*10];
theta_joint_ceiling=[pi/6,pi*3/2,pi,pi*10];
d_joint=[0,1,0,0];
a_joint=[0,0,3,3];
alpha_joint=[0,-pi/2,0,0];
for i = 1:num_joint
    L(i)=Link([d_joint(i),theta_joint(1,i),a_joint(i),alpha_joint(i)],'modified');
end
Leg=SerialLink(L);
Leg.qlim=[theta_joint_floor.',theta_joint_ceiling.'];

f_w_joint_min=pi/(4*shiftrange);
K_num=10;
K_f_w_joint=(1:K_num);
joint=t_ecd.'*f_w_joint_min*K_f_w_joint;

K1=[-0.092175204 0.055156594 0.228888365 0.200556885 0.350061213 0.672317906 0.394845532 -0.482850929 -0.859851211 -0.419951884].';
K2=[0.680682733 0.296941315 0.263194088 0.381940112 0.293745207 0.151310506 0.210632043 0.212464916 -0.014183956 0.151397236].';
K3=zeros(K_num,1);
w_joint1=sin(joint)*K1;
w_joint2=cos(joint)*K2;
w_joint3=cos(joint)*K3;
w_joint4=zeros(length(t_ecd), 1);
w_joint=[w_joint1,w_joint2,w_joint3,w_joint4];

for i=2:length(t_ecd)
    theta_joint(i,:)=theta_joint(i-1,:)+w_joint(i-1,:)*dt_ecd;
end
theta1range=max(theta_joint(:,1))-min(theta_joint(:,1));
theta2range=max(theta_joint(:,2))-min(theta_joint(:,2));

w_imu_c=zeros(length(t_ecd),3);
w_effctor=[w_joint1.*sin(theta_joint(:,2)).*sin(theta_joint(:,3)).*sin(theta_joint(:,4))-...
        w_joint1.*cos(theta_joint(:,2)).*sin(theta_joint(:,3)).*cos(theta_joint(:,4))-...
        w_joint1.*sin(theta_joint(:,2)).*cos(theta_joint(:,3)).*cos(theta_joint(:,4))-...
        w_joint1.*cos(theta_joint(:,2)).*cos(theta_joint(:,3)).*sin(theta_joint(:,4)),...
              w_joint1.*cos(theta_joint(:,2)).*sin(theta_joint(:,3)).*sin(theta_joint(:,4))-...
        w_joint1.*cos(theta_joint(:,2)).*cos(theta_joint(:,3)).*cos(theta_joint(:,4))+...
        w_joint1.*sin(theta_joint(:,2)).*cos(theta_joint(:,3)).*sin(theta_joint(:,4))+...
        w_joint1.*sin(theta_joint(:,2)).*sin(theta_joint(:,3)).*cos(theta_joint(:,4)),...
              w_joint2+w_joint3+w_joint4];

f_imu_g=f_ecd;
w_imu_i=w_effctor;
K_bg_imu_g=0.1;
rng(0);
bg_imu_g=2*K_bg_imu_g*rand(1,3)-K_bg_imu_g;
K_ng_imu_g=10;
rng(5);  
theta_ig=360*rand(1,3);
R_ig=eul2rotm(deg2rad(theta_ig));
w_imu_g=w_imu_i*R_ig.';
w_imu_g_read = awgn(w_imu_g,K_ng_imu_g)+bg_imu_g;

ts_ecd_1=0.45;
ps_ecd_1=round(ts_ecd_1*f_ecd);
ts_ecd_2=-0.23;
ps_ecd_2=round(ts_ecd_2*f_ecd);
ts_ecd_3=0;
ps_ecd_3=round(ts_ecd_3*f_ecd);
pd_ecd_3=0;
slidewindow=8*shiftrange;
starttime=2;
startposition=round(starttime*f_ecd);

epsilon_b = 0.9;
zeta_b = 0.015;
zeta_u = 20;

G=w_imu_g_read(startposition:startposition+slidewindow*f_ecd,:);
Gx=G(:,1)-mean(G(:,1));
Gy=G(:,2)-mean(G(:,2));
Gz=G(:,3)-mean(G(:,3));
sigma_gg=[mean(Gx.*Gx),mean(Gx.*Gy),mean(Gx.*Gz);
          mean(Gy.*Gx),mean(Gy.*Gy),mean(Gy.*Gz);
          mean(Gz.*Gx),mean(Gz.*Gy),mean(Gz.*Gz)];
r_ig_td=zeros(2*shiftrange*f_ecd+1,2*shiftrange*f_ecd+1);
theta_joint_read=zeros(slidewindow*f_ecd+1,num_joint);
w_joint_window=zeros(slidewindow*f_ecd+1,num_joint);
tic
for td_ecd_1=-shiftrange:dt_ecd:shiftrange
    pd_ecd_1=round(td_ecd_1*f_ecd);
    for td_ecd_2=-shiftrange:dt_ecd:shiftrange
        pd_ecd_2=round(td_ecd_2*f_ecd);
        theta_joint_read(:,1)=theta_joint(startposition-ps_ecd_1+pd_ecd_1 : startposition-ps_ecd_1+pd_ecd_1+slidewindow*f_ecd,1);
        theta_joint_read(:,2)=theta_joint(startposition-ps_ecd_2+pd_ecd_2 : startposition-ps_ecd_2+pd_ecd_2+slidewindow*f_ecd,2);
        theta_joint_read(:,3)=theta_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,3);
        theta_joint_read(:,4)=theta_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,4);
        w_joint_window(:,1)=w_joint(startposition-ps_ecd_1+pd_ecd_1 : startposition-ps_ecd_1+pd_ecd_1+slidewindow*f_ecd,1);
        w_joint_window(:,2)=w_joint(startposition-ps_ecd_2+pd_ecd_2 : startposition-ps_ecd_2+pd_ecd_2+slidewindow*f_ecd,2);
        w_joint_window(:,3)=w_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,3);
        w_joint_window(:,4)=w_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,4);
        I=[w_joint_window(:,1).*sin(theta_joint_read(:,2)).*sin(theta_joint_read(:,3)).*sin(theta_joint_read(:,4))-...
            w_joint_window(:,1).*cos(theta_joint_read(:,2)).*sin(theta_joint_read(:,3)).*cos(theta_joint_read(:,4))-...
            w_joint_window(:,1).*sin(theta_joint_read(:,2)).*cos(theta_joint_read(:,3)).*cos(theta_joint_read(:,4))-...
            w_joint_window(:,1).*cos(theta_joint_read(:,2)).*cos(theta_joint_read(:,3)).*sin(theta_joint_read(:,4)),...
           w_joint_window(:,1).*cos(theta_joint_read(:,2)).*sin(theta_joint_read(:,3)).*sin(theta_joint_read(:,4))-...
            w_joint_window(:,1).*cos(theta_joint_read(:,2)).*cos(theta_joint_read(:,3)).*cos(theta_joint_read(:,4))+...
            w_joint_window(:,1).*sin(theta_joint_read(:,2)).*cos(theta_joint_read(:,3)).*sin(theta_joint_read(:,4))+...
            w_joint_window(:,1).*sin(theta_joint_read(:,2)).*sin(theta_joint_read(:,3)).*cos(theta_joint_read(:,4)),...
           w_joint_window(:,2)+w_joint_window(:,3)+w_joint_window(:,4)];
        Ix=I(:,1)-mean(I(:,1));
        Iy=I(:,2)-mean(I(:,2));
        Iz=I(:,3)-mean(I(:,3));
        sigma_gi=[mean(Gx.*Ix),mean(Gx.*Iy),mean(Gx.*Iz);
                  mean(Gy.*Ix),mean(Gy.*Iy),mean(Gy.*Iz);
                  mean(Gz.*Ix),mean(Gz.*Iy),mean(Gz.*Iz)];
        sigma_ii=[mean(Ix.*Ix),mean(Ix.*Iy),mean(Ix.*Iz);
                  mean(Iy.*Ix),mean(Iy.*Iy),mean(Iy.*Iz);
                  mean(Iz.*Ix),mean(Iz.*Iy),mean(Iz.*Iz)];
        sigma_ig=[mean(Ix.*Gx),mean(Ix.*Gy),mean(Ix.*Gz);
                  mean(Iy.*Gx),mean(Iy.*Gy),mean(Iy.*Gz);
                  mean(Iz.*Gx),mean(Iz.*Gy),mean(Iz.*Gz)];
        r_ig_td(round(f_ecd*td_ecd_1+f_ecd*shiftrange+1),round(f_ecd*td_ecd_2+f_ecd*shiftrange+1))=(1/3*trace(sigma_gg^-1*sigma_gi*sigma_ii^-1*sigma_ig))^0.5;
    end
end
timeconsume=toc;
[r_ig_td_max,linearIndex]=max(r_ig_td(:));
[row,col]=ind2sub(size(r_ig_td),linearIndex);
ts_ecd_1_estimate=(row-shiftrange*f_ecd-1)/f_ecd;
ts_ecd_2_estimate=(col-shiftrange*f_ecd-1)/f_ecd;
axis_x=linspace(-shiftrange,shiftrange,2*shiftrange*f_ecd+1);
axis_y=linspace(-shiftrange,shiftrange,2*shiftrange*f_ecd+1);
figure,mesh(axis_x,axis_y,r_ig_td);
xlabel('td_ecd_2');
ylabel('td_ecd_1');
zlabel('r_ig_td');
hold on;
scatter3(axis_x(col), axis_y(row), r_ig_td_max, 'r', 'filled');
hold off;
ps_ecd_1_estimate=round(ts_ecd_1_estimate*f_ecd);
ps_ecd_2_estimate=round(ts_ecd_2_estimate*f_ecd);
theta_joint_correct(:,1)=theta_joint(startposition-ps_ecd_1+ps_ecd_1_estimate : startposition-ps_ecd_1+ps_ecd_1_estimate+slidewindow*f_ecd,1);
theta_joint_correct(:,2)=theta_joint(startposition-ps_ecd_2+ps_ecd_2_estimate : startposition-ps_ecd_2+ps_ecd_2_estimate+slidewindow*f_ecd,2);
theta_joint_correct(:,3)=theta_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,3);
theta_joint_correct(:,4)=theta_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,4);
w_joint_correct(:,1)=w_joint(startposition-ps_ecd_1+ps_ecd_1_estimate : startposition-ps_ecd_1+ps_ecd_1_estimate+slidewindow*f_ecd,1);
w_joint_correct(:,2)=w_joint(startposition-ps_ecd_2+ps_ecd_2_estimate : startposition-ps_ecd_2+ps_ecd_2_estimate+slidewindow*f_ecd,2);
w_joint_correct(:,3)=w_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,3);
w_joint_correct(:,4)=w_joint(startposition-ps_ecd_3+pd_ecd_3 : startposition-ps_ecd_3+pd_ecd_3+slidewindow*f_ecd,4);
I_correct=[w_joint_correct(:,1).*sin(theta_joint_correct(:,2)).*sin(theta_joint_correct(:,3)).*sin(theta_joint_correct(:,4))-...
    w_joint_correct(:,1).*cos(theta_joint_correct(:,2)).*sin(theta_joint_correct(:,3)).*cos(theta_joint_correct(:,4))-...
    w_joint_correct(:,1).*sin(theta_joint_correct(:,2)).*cos(theta_joint_correct(:,3)).*cos(theta_joint_correct(:,4))-...
    w_joint_correct(:,1).*cos(theta_joint_correct(:,2)).*cos(theta_joint_correct(:,3)).*sin(theta_joint_correct(:,4)),...
   w_joint_correct(:,1).*cos(theta_joint_correct(:,2)).*sin(theta_joint_correct(:,3)).*sin(theta_joint_correct(:,4))-...
    w_joint_correct(:,1).*cos(theta_joint_correct(:,2)).*cos(theta_joint_correct(:,3)).*cos(theta_joint_correct(:,4))+...
    w_joint_correct(:,1).*sin(theta_joint_correct(:,2)).*cos(theta_joint_correct(:,3)).*sin(theta_joint_correct(:,4))+...
    w_joint_correct(:,1).*sin(theta_joint_correct(:,2)).*sin(theta_joint_correct(:,3)).*cos(theta_joint_correct(:,4)),...
   w_joint_correct(:,2)+w_joint_correct(:,3)+w_joint_correct(:,4)];
Ix_correct=I_correct(:,1)-mean(I_correct(:,1));
Iy_correct=I_correct(:,2)-mean(I_correct(:,2));
Iz_correct=I_correct(:,3)-mean(I_correct(:,3));
sigma_ii_correct=[mean(Ix_correct.*Ix_correct),mean(Ix_correct.*Iy_correct),mean(Ix_correct.*Iz_correct);
                  mean(Iy_correct.*Ix_correct),mean(Iy_correct.*Iy_correct),mean(Iy_correct.*Iz_correct);
                  mean(Iz_correct.*Ix_correct),mean(Iz_correct.*Iy_correct),mean(Iz_correct.*Iz_correct)];
sigma_ig_correct=[mean(Ix_correct.*Gx),mean(Ix_correct.*Gy),mean(Ix_correct.*Gz);
                  mean(Iy_correct.*Gx),mean(Iy_correct.*Gy),mean(Iy_correct.*Gz);
                  mean(Iz_correct.*Gx),mean(Iz_correct.*Gy),mean(Iz_correct.*Gz)];
[U,S,V]=svd(sigma_ii_correct^-1*sigma_ig_correct);
R_gi_estimate=(U*[1,0,0;0,1,0;0,0,det(U*V.')]*V.');
theta_ig_estimate = roundn(rotm2eul(R_gi_estimate.')/pi*180,-2)+[0,360,0];
figure;
subplot(2,2,1);plot(t_ecd,theta_joint(:,1));
subplot(2,2,2);plot(t_ecd,theta_joint(:,2));
subplot(2,2,3);plot(t_ecd,w_joint1);
subplot(2,2,4);plot(t_ecd,w_joint2);

figure;
subplot(1,3,1);plot(t_ecd, w_effctor(:,1));
subplot(1,3,2);plot(t_ecd, w_effctor(:,2));
subplot(1,3,3);plot(t_ecd, w_effctor(:,3));
cond_num_sigma_gg=cond(sigma_gg);
lambda_min=abs(min(eig(sigma_gg)));
disp(["IMU信噪比：",K_ng_imu_g,'dB']);
disp(["估计时间偏移用时：",timeconsume,'s']);
disp(["关节1运动最大角度：",theta1range; ...
      "关节2运动最大角度：",theta2range]);
disp("sigma_gg =");
disp(sigma_gg);
if r_ig_td_max>epsilon_b && lambda_min>zeta_b && cond_num_sigma_gg<zeta_u
    disp("满足可观测性条件")
else
    disp("不满足可观测性条件")
end
disp(["r_ig_td_max=",r_ig_td_max,"epsilon_b=",epsilon_b; ...
    "abs_lambda_min=",lambda_min,"zeta_b=",zeta_b; ...
    "sigma_gg矩阵条件数=",cond_num_sigma_gg, "zeta_u =", zeta_u]);
disp(["编码器1时移估计：",ts_ecd_1_estimate, "编码器1时移估计误差：",1000*(ts_ecd_1_estimate-ts_ecd_1),'ms'; ...
      "编码器2时移估计：",ts_ecd_2_estimate, "编码器2时移估计误差：",1000*(ts_ecd_2_estimate-ts_ecd_2),'ms']);
disp(["旋转矩阵估计：","           旋转矩阵估计误差:"]);
disp([R_gi_estimate,R_gi_estimate-R_ig.']);
disp(["旋转角度估计：",theta_ig_estimate,"旋转角度估计误差：",theta_ig_estimate-theta_ig]);