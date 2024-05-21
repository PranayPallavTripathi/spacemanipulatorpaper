clear; clc; 
joint_info = readmatrix('ifile_joint2.txt'); % dim(joint_info) = [N+1 X 4]
%including base-spacecraft COM, no of joints is N+1 
% where there are N links (excluding base-spacecraft), then, no_of_joints represents N
no_of_joints = (height(joint_info)-1); % dim(no_of_joints) = [1 X 1]; no_of_joints = N

J_T_J0_info = readmatrix('JT0_inputs.txt'); % dim(J_T_J0_info) = [1 X 6] for base-spacecraft
link_info = readmatrix('ifile_link2.txt'); % dim(link_info) = [N+1 X 4]

J_T_J0 = JTJ0(J_T_J0_info); % dim(J_T_J0) = [4 X 4]

mass_info = readmatrix('ifile_mass.txt'); % dim(mass_info) = [1 X N+1] for base spacecraft and N links
moi_info = readmatrix('ifile_moi.txt'); % dim(moi_info) = [3 X 3*(N+1)] for base spacecraft and N links
moi_info = reshape(moi_info,3,3,no_of_joints+1); % dim(moi_info) = [3 X 3 x (N+1)] for base spacecraft and N links

time_step = 0.01;
threshold=1e-6;
% torque_time = readmatrix('ifile_torque_time.txt');
qD_dot_time = readmatrix('ifile_qDdot.txt');
end_time = qD_dot_time(height(qD_dot_time),width(qD_dot_time));

t = datetime('now');
t = string(t);
t = replace(t,":","-");
filename = "joint_angle " + string(t) + ".txt";
filename_1 = "joint_angle_dot " + string(t) + ".txt";
filename_2 = "J_x0_Jx0_dot " + string(t) + ".txt";
filename_3 = "System_COM " + string(t) + ".txt";
fileID = fopen(filename,'w');
fileID_1 = fopen(filename_1,'w');
fileID_2 = fopen(filename_2,'w');
fileID_3 = fopen(filename_3,'w');

m_tot = sum(mass_info);



H_star = zeros(no_of_joints,no_of_joints);
c_star = zeros(no_of_joints,1);
H_star_dot = zeros(no_of_joints,no_of_joints);
q = zeros(no_of_joints,1);
q_dot = zeros(no_of_joints,1);
C_star = H_star_dot*q_dot + c_star;

q_dot_dot_prev1=zeros(no_of_joints,1);
q_dot_dot_prev2=zeros(no_of_joints,1);
q_dot_prev1=zeros(no_of_joints,1);
q_dot_prev2=zeros(no_of_joints,1);
q_prev1=zeros(no_of_joints,1);
q_prev2=zeros(no_of_joints,1);
J_x0_dot_prev1=zeros(6,1);
J_x0_dot_prev2=zeros(6,1);
J_x0_prev1=zeros(6,1);
J_x0_prev2=zeros(6,1);
euler_dot_prev2=zeros(3,1);
euler_dot_prev1=zeros(3,1);
euler_prev2=zeros(3,1);
euler_prev1=zeros(3,1);

qD_dot_prev1=zeros(no_of_joints,1);
qD_dot_prev2=zeros(no_of_joints,1);
qD_prev1=zeros(no_of_joints,1);
qD_prev2=zeros(no_of_joints,1);



% t = 0;
% row_torque_time = 1;
% for n=1:1:floor(end_time/time_step)+1
%     if(t-torque_time(row_torque_time,width(torque_time))>0.00001)
%         row_torque_time = row_torque_time + 1; 
%     end
%     joint_torque(1:no_of_joints,n) = torque_time(row_torque_time,1:no_of_joints)';
%     t = t+time_step;
% end

%%%% Control Loop %%%%


K_P = zeros(no_of_joints,1);
K_D = ones(no_of_joints,1);

t = 0;
row_qD_dot_time = 1;
for n=1:1:round(end_time/time_step)+1
    if(t-qD_dot_time(row_qD_dot_time,width(qD_dot_time))>0.00001)
        row_qD_dot_time = row_qD_dot_time + 1; 
    end
    joint_qD_dot(1:no_of_joints,n) = qD_dot_time(row_qD_dot_time,1:no_of_joints)';
    t = t+time_step;
end



for time = 0:time_step:end_time 


%%%%    Control loop    %%%%
qD_dot = joint_qD_dot(:,round(time/time_step)+1);
if time==0
    qD=zeros(no_of_joints,1);
elseif time==time_step
        qD=((time_step)/6) * (qD_dot_prev1 + 2*(qD_dot_prev1+qD_dot) + qD_dot);
else
    qD=simpson((time-2*time_step),time,qD_dot_prev2,qD_dot_prev1,qD_dot)+qD_prev2;
end

for i=1:1:length(qD)
    if abs(qD(i))<threshold
        qD(i)=0;
    end
end

qD_dot_prev2=qD_dot_prev1;
qD_dot_prev1=qD_dot;
qD_prev2=qD_prev1;
qD_prev1=qD;

u_bar = K_D.*(qD_dot - q_dot) + K_P.*(qD - q);

joint_torque = H_star*u_bar + C_star;



%%%%    Spacecraft manipulator equations    %%%%
for i=1:1:no_of_joints+1 %from J0_T_J1 to JN_T_JN+1
    Ji_T_Jiplus1(:,:,i)=Ji_A_Jiplus1(joint_info(i,:)); % dim(Ji_T_Jiplus1) = [4 X 4 X no_of_joints+1]
end

for i=1:1:no_of_joints+2 % J_T_J0 to J_T_JNplus1
    J_T_Ji(:,:,i)=J_T_J0;
    for j=1:1:i-1
        J_T_Ji(:,:,i) = J_T_Ji(:,:,i)*Ji_T_Jiplus1(:,:,j); % dim(J_T_Ji) = [4 X 4 X no_of_joints+2]
    end
end

for i=1:1:no_of_joints %from J1_T_L1 to JN_T_LN
    Ji_T_Li(:,:,i)=Ji_A_Jiplus1(link_info(i+1,:)); % dim(Ji_T_Li) = [4 X 4 X no_of_joints]
end

for i=1:1:no_of_joints+1 % J_T_L0 to J_T_LN
    if i==1
        J_T_Li(:,:,i)=J_T_J0;
    else
        J_T_Li(:,:,i) = J_T_Ji(:,:,i)*Ji_T_Li(:,:,i-1); % dim(J_T_Li) = [4 X 4 X no_of_joints+1]
    end
end

for i=1:1:no_of_joints+2 % i=1 stands for base space craft and i=no_of_joints+2 stands for end effector
    J_R_Ji(:,:,i)=J_T_Ji(1:3,1:3,i); % dim(J_R_Ji) = [3 x 3 x no_of_joints+2]  i=1 defines base space craft -> J0
    J_k_i(:,i)=J_R_Ji(:,:,i)*[0;0;1]; % dim(J_k_i) = [3 x 1 x no_of_joints+2] J_k_i_cap
    J_p_i(:,i)=J_T_Ji(1:3,4,i);
    if i~=no_of_joints+2
        J_r_i(:,i)=J_T_Li(1:3,4,i);
        J_r_0i(:,i)=J_r_i(:,i)-J_r_i(:,1); %J_r_00 to J_r_0N
    end
end 

J_r_C=J_r_i(:,1);
for i=2:no_of_joints+1
    J_r_C = J_r_C + (1/m_tot)*(mass_info(i)*J_r_0i(:,i));
end

fprintf(fileID_3,'%12.8f %12.8f %12.8f \n', J_r_C(1), J_r_C(2), J_r_C(3));

% subplot(6,1,5)
% scatter(time,J_r_C(1),'.','r'); hold on;
% subplot(6,1,6)
% scatter(time,J_r_C(2),'.','r'); hold on;
J_r_0C = J_r_C - J_r_i(:,1);

%%%%    Spacecraft manipulator dynamics    %%%%

J_Ri=zeros(3,no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1 % J_R1 to J_RN
    for j=2:1:i
        J_Ri(:,j-1,i-1)=J_k_i(:,j);
        %J_Ri(:,j-1,i-1)=J_k_i(:,j-1);
    end
end


for i=2:1:no_of_joints+1
    J_I_i(:,:,i-1)=J_R_Ji(:,:,i+1)*moi_info(:,:,i)*J_R_Ji(:,:,i+1)'; % J_I_1 to J_I_N
end
%J_I_i

J_Ti=zeros(3,no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1 % J_T1 to J_TN
    for j=2:1:i
        J_Ti(:,j-1,i-1)=vector2skew(J_k_i(:,j))*(J_r_i(:,i)-J_p_i(:,j));
        %J_Ti(:,j-1,i-1)=vector2skew(J_k_i(:,j-1))*(J_r_i(:,i)-J_p_i(:,j-1));
    end
end

Hm=zeros(no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1
    Hm = Hm + J_Ri(:,:,i-1)'*J_I_i(:,:,i-1)*J_Ri(:,:,i-1) + mass_info(i)*J_Ti(:,:,i-1)'*J_Ti(:,:,i-1);
end

J_TS=zeros(3,no_of_joints);
for i=2:1:no_of_joints+1
    J_TS = J_TS + mass_info(i)*J_Ti(:,:,i-1);
end

H_Sq=zeros(3,no_of_joints);
for i=2:1:no_of_joints+1
    H_Sq = H_Sq + J_I_i(:,:,i-1)*J_Ri(:,:,i-1) + mass_info(i)*vector2skew(J_r_0i(:,i))*J_Ti(:,:,i-1);
end

H0m=[J_TS;H_Sq];

H0=zeros(6,6);
H0(1:3,1:3)=m_tot*eye(3);
H0(1:3,4:6)=-m_tot*vector2skew(J_r_0C);
H0(4:6,1:3)=-H0(1:3,4:6);
HS=J_R_Ji(:,:,1)*moi_info(:,:,1)*J_R_Ji(:,:,1)';
for i=2:1:no_of_joints+1
    HS=HS+J_I_i(:,:,i-1)-mass_info(i)*vector2skew(J_r_0i(:,i))*vector2skew(J_r_0i(:,i));
end
H0(4:6,4:6)=HS;
H_star = Hm - H0m'*(H0\H0m);


% joint_torque(:,round(time/time_step)+1) 
% C_star
%inv(H_star)
% q_dot_dot = H_star\(joint_torque(:,round(time/time_step)+1) - C_star);
q_dot_dot = H_star\(joint_torque - C_star);
% c_star

%%%%% simpson integration

for i=1:1:length(q_dot_dot)
    if abs(q_dot_dot(i))<threshold
        q_dot_dot(i)=0;
    end
end

if time==0
    q_dot=zeros(no_of_joints,1);
elseif time==time_step
        q_dot=((time_step)/6) * (q_dot_dot_prev1 + 2*(q_dot_dot_prev1+q_dot_dot) + q_dot_dot);
else
    q_dot=simpson((time-2*time_step),time,q_dot_dot_prev2,q_dot_dot_prev1,q_dot_dot)+q_dot_prev2;
end

for i=1:1:length(q_dot)
    if abs(q_dot(i))<threshold
        q_dot(i)=0;
    end
end

if time==0
    q=zeros(no_of_joints,1);
elseif time==time_step
        q=((time_step)/6) * (q_dot_prev1 + 2*(q_dot_prev1+q_dot) + q_dot);
else
    q=simpson((time-2*time_step),time,q_dot_prev2,q_dot_prev1,q_dot)+q_prev2;
end

for i=1:1:length(q)
    if abs(q(i))<threshold
        q(i)=0;
    end
end

q_dot_dot_prev2=q_dot_dot_prev1;
q_dot_dot_prev1=q_dot_dot;
q_dot_prev2=q_dot_prev1;
q_dot_prev1=q_dot;
q_prev2=q_prev1;
q_prev1=q;

fprintf(fileID,'%12.4f %12.4f %12.4f %12.4f\n',q(1), q(2), q(3), q(4));
fprintf(fileID_1,'%12.4f %12.4f %12.4f %12.4f\n',q_dot(1), q_dot(2), q_dot(3), q_dot(4));

% subplot(6,1,1)
% scatter(time,q(1),'.','r'); hold on;
% subplot(6,1,2)
% scatter(time,q(2),'.','r'); hold on;
% subplot(6,1,3)
% scatter(time,q(3),'.','r'); hold on;
% subplot(6,1,4)
% scatter(time,q(4),'.','r'); hold on;

%%% Degeneralisation eqn. 109
J_x0_dot = -1*H0\H0m*q_dot;
J_v_0 = J_x0_dot(1:3);
J_w_0 = J_x0_dot(4:6);

for i=1:1:length(J_x0_dot)
    if abs(J_x0_dot(i))<threshold
        J_x0_dot(i)=0;
    end
end

if time==0
    J_x0=zeros(6,1);
elseif time==time_step
        J_x0=((time_step)/6) * (J_x0_dot_prev1 + 2*(J_x0_dot_prev1+J_x0_dot) + J_x0_dot);
else
    J_x0=simpson((time-2*time_step),time,J_x0_dot_prev2,J_x0_dot_prev1,J_x0_dot)+J_x0_prev2;
end

for i=1:1:length(J_x0)
    if abs(J_x0(i))<threshold
        J_x0(i)=0;
    end
end

J_x0_dot_prev2=J_x0_dot_prev1;
J_x0_dot_prev1=J_x0_dot;
J_x0_prev2=J_x0_prev1;
J_x0_prev1=J_x0;


% calculating angular velocities in the inertial frame
J_w_Ji(:,1) = J_w_0;
for i=2:1:no_of_joints+1
    J_w_Ji(:,i) = J_w_Ji(:,i-1);
    for j=2:1:i
        J_w_Ji(:,i) = J_w_Ji(:,i) + J_k_i(:,j)*q_dot(j-1);    %%% to find Last link ang. vel., end effector movement
    end                                                       %%% taken same as last nth joint movement
end
J_w_Ji(:,no_of_joints+2) = J_w_Ji(:,no_of_joints+1);

%J_R_J0_dot to J_R_JN_dot
J_R_Ji_dot(:,:,1)=vector2skew(J_w_0)*J_R_Ji(:,:,1);
for i=2:1:no_of_joints+2
    J_R_Ji_dot(:,:,i)=vector2skew(J_w_Ji(:,i))*J_R_Ji(:,:,i);    
end                                                               

J_Ri_dot=zeros(3,no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1 % J_R1_dot to J_RN_dot
    for j=2:1:no_of_joints+1
        if j<=i
            J_Ri_dot(:,j-1,i-1)=J_R_Ji_dot(:,:,j)*[0;0;1];
        end
    end
end

% calculating link velocities in the inertial frame
J_r_i_dot(:,1) = J_v_0;
for i=2:1:no_of_joints+1 % J_r_1_dot to J_r_N_dot
    J_r_i_dot(:,i) = J_v_0 + vector2skew(J_w_0)*J_r_0i(:,i);
    for j=2:1:i
        J_r_i_dot(:,i)=J_r_i_dot(:,i) + vector2skew(J_k_i(:,j)) * (J_r_i(:,i) - J_p_i(:,j)) * q_dot(j-1);
    end
end

% calculating joint velocities in the inertial frame
J_p_i_dot(:,1) = J_v_0;
for i=2:1:no_of_joints+1 
    J_p_i_dot(:,i) = J_v_0 + vector2skew(J_w_0)*(J_p_i(:,i) - J_r_i(:,1));
    for j=2:1:i
        J_p_i_dot(:,i)=J_p_i_dot(:,i) + vector2skew(J_k_i(:,j)) * (J_p_i(:,i) - J_p_i(:,j)) * q_dot(j-1);
    end
end


J_Ti_dot=zeros(3,no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1 % J_T1_dot to J_TN_dot
    for j=2:1:no_of_joints+1
        if j<=i
            J_Ti_dot(:,j-1,i-1)=(vector2skew(J_R_Ji_dot(:,:,j)*[0;0;1]) * (J_r_i(:,i)-J_p_i(:,j))) ...
                                + (vector2skew(J_k_i(:,j)) * (J_r_i_dot(:,i) - J_p_i_dot(:,j)));
        end
    end
end

% calculating time derivative of MOI matrix in inertial frame
for i=2:1:no_of_joints+1
    J_I_i_dot(:,:,i-1) = J_R_Ji_dot(:,:,i+1)*moi_info(:,:,i)*J_R_Ji(:,:,i+1)' + J_R_Ji(:,:,i+1)*moi_info(:,:,i)*J_R_Ji_dot(:,:,i+1)';
end


Hm_dot = zeros(no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1
    Hm_dot = Hm_dot + J_Ri_dot(:,:,i-1)'*J_I_i(:,:,i-1)*J_Ri(:,:,i-1) ...
                    + J_Ri(:,:,i-1)'*J_I_i_dot(:,:,i-1)*J_Ri(:,:,i-1) ...
                    + J_Ri(:,:,i-1)'*J_I_i(:,:,i-1)*J_Ri_dot(:,:,i-1) ...
                    + mass_info(i) * ((J_Ti_dot(:,:,i-1)'*J_Ti(:,:,i-1))+(J_Ti(:,:,i-1)'*J_Ti_dot(:,:,i-1)));
end

% J_r_0c_dot = zeros(3,1);
m_tot_J_r_0c_dot_x = zeros(3,1);
for i=2:1:no_of_joints+1
    m_tot_J_r_0c_dot_x = m_tot_J_r_0c_dot_x + mass_info(i)*(vector2skew(J_r_i_dot(:,i)-J_v_0));
end

HS_dot = J_R_Ji_dot(:,:,1)*moi_info(:,:,1)*J_R_Ji(:,:,1)'+J_R_Ji(:,:,1)*moi_info(:,:,1)*J_R_Ji_dot(:,:,1)';  %%% HS_dot initialized as J_I_0_dot

for i=2:1:no_of_joints+1
    HS_dot = HS_dot + J_I_i_dot(:,:,i-1) - mass_info(i)*((vector2skew(J_r_i_dot(:,i)-J_v_0))*vector2skew(J_r_0i(:,i))  ... 
                                                        + (vector2skew(J_r_0i(:,i))*vector2skew(J_r_i_dot(:,i)-J_v_0)));
end

H0_dot = [zeros(3,3), -m_tot_J_r_0c_dot_x; m_tot_J_r_0c_dot_x, HS_dot];

J_TS_dot = zeros(3,no_of_joints);
for i = 2:1:no_of_joints
   J_TS_dot = J_TS_dot + mass_info(i)*J_Ti_dot(:,:,i-1);
end

H_Sq_dot=zeros(3,no_of_joints);
for i=2:1:no_of_joints+1
    H_Sq_dot = H_Sq_dot + J_I_i_dot(:,:,i-1)*J_Ri(:,:,i-1) + J_I_i(:,:,i-1)*J_Ri_dot(:,:,i-1) ...
                        + mass_info(i)*vector2skew(J_r_0i(:,i))*J_Ti_dot(:,:,i-1) ...
                        + mass_info(i)*vector2skew(J_r_i_dot(:,i)-J_v_0)*J_Ti(:,:,i-1);
end

H0m_dot = [J_TS_dot; H_Sq_dot];

H_star_dot = Hm_dot - (H0m_dot'*(H0\H0m) + H0m'*(H0\H0m_dot) - H0m'*(H0\H0_dot)*(H0\H0m));

%%%  Determination of c_star   %%%


J_T_Ji_qk = zeros(4,4,no_of_joints+2,no_of_joints);

for i = 1:1:no_of_joints+2
    for k = 2:1:no_of_joints+1
        if(k-1==i-2)
            J_T_Ji_qk(:,:,i,k-1) = J_T_Ji(:,:,i-1)*A_qk(joint_info(i-1,:)); 
        elseif ((k-1)<(i-2))
            J_T_Ji_qk(:,:,i,k-1) = J_T_Ji(:,:,k)*A_qk(joint_info(k,:));
            for l=k+1:1:i-1
                J_T_Ji_qk(:,:,i,k-1) = J_T_Ji_qk(:,:,i,k-1)*Ji_A_Jiplus1(joint_info(l,:));
            end
        end
    end
end

J_T_Li_qk = zeros(4,4,no_of_joints+2,no_of_joints);

for i = 1:1:no_of_joints+1
    for k = 2:1:no_of_joints+1
        if(k-1==i-1)
            J_T_Li_qk(:,:,i,k-1) = J_T_Ji(:,:,i)*A_qk(link_info(i,:));
        elseif ((k-1)<(i-1))
            J_T_Li_qk(:,:,i,k-1) = J_T_Ji_qk(:,:,i,k-1)*Ji_A_Jiplus1(link_info(i,:));
        end
    end
end

J_Ri_qk = zeros(3,no_of_joints,no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1
    for k=2:1:no_of_joints+1
        for l=2:1:no_of_joints+1
            if l<=i
                J_Ri_qk(:,l-1,i-1,k-1) = J_T_Ji_qk(1:3,1:3,l,k-1)*[0;0;1];
            end
        end
    end
end

J_Ti_qk = zeros(3,no_of_joints,no_of_joints,no_of_joints);
for i=2:1:no_of_joints+1
    for k=2:1:no_of_joints+1
        for l=2:1:no_of_joints+1
            if l<=i
                J_Ti_qk(:,l-1,i-1,k-1) = vector2skew(J_T_Ji_qk(1:3,1:3,l,k-1)*[0;0;1])*(J_r_i(:,i)-J_p_i(:,l)) + ...
                                         vector2skew(J_k_i(:,l))*(J_T_Li_qk(1:3,4,i,k-1)-J_T_Ji_qk(1:3,4,l,k-1));
            end
        end
    end
end

for i=2:1:no_of_joints+1
    for k=2:1:no_of_joints+1
        J_I_i_qk(:,:,i-1,k-1) = J_T_Li_qk(1:3,1:3,i,k-1)*moi_info(:,:,i)*J_T_Li(1:3,1:3,i)' + ...
                                J_T_Li(1:3,1:3,i)*moi_info(:,:,i)*J_T_Li_qk(1:3,1:3,i,k-1)';
    end
end

Hm_qk = zeros(no_of_joints,no_of_joints,no_of_joints);
for k=2:1:no_of_joints+1
    for i=2:1:no_of_joints+1
        Hm_qk(:,:,k-1) = Hm_qk(:,:,k-1) + J_Ri_qk(:,:,i-1,k-1)'*J_I_i(:,:,i-1)*J_Ri(:,:,i-1) ...
                                    + J_Ri(:,:,i-1)'*J_I_i_qk(:,:,i-1,k-1)*J_Ri(:,:,i-1) ...
                                    + J_Ri(:,:,i-1)'*J_I_i(:,:,i-1)*J_Ri_qk(:,:,i-1,k-1) ...
                                    + mass_info(i)*J_Ti_qk(:,:,i-1,k-1)'*J_Ti(:,:,i-1) ...
                                    + mass_info(i)*J_Ti(:,:,i-1)'*J_Ti_qk(:,:,i-1,k-1);
    end
end

J_TS_qk = zeros(3,no_of_joints,no_of_joints);
for k=2:1:no_of_joints+1
    for i=2:1:no_of_joints+1
        J_TS_qk(:,:,k-1) = J_TS_qk(:,:,k-1) + mass_info(i)*J_Ti_qk(:,:,i-1,k-1);
    end
end

H_Sq_qk = zeros(3,no_of_joints,no_of_joints);
for k=2:1:no_of_joints+1
    for i=2:1:no_of_joints+1
        H_Sq_qk(:,:,k-1) = H_Sq_qk(:,:,k-1) + J_I_i_qk(:,:,i-1,k-1)*J_Ri(:,:,i-1) ...
                                            + J_I_i(:,:,i-1)*J_Ri_qk(:,:,i-1,k-1) ...
                                            + mass_info(i)*vector2skew(J_T_Li_qk(1:3,4,i,k-1))*J_Ti(:,:,i-1) ...
                                            + mass_info(i)*vector2skew(J_r_0i(:,i))*J_Ti_qk(:,:,i-1,k-1);
    end
end

H0m_qk = [J_TS_qk; H_Sq_qk];

r0C_qk = zeros(3, no_of_joints);
for k=2:1:no_of_joints+1
    for i=2:1:no_of_joints+1
        r0C_qk(:,k-1) = r0C_qk(:,k-1) + mass_info(i)*J_T_Li_qk(1:3,4,i,k-1);
    end
end
r0C_qk = r0C_qk/m_tot;

HS_qk = zeros(3,3,no_of_joints);
for k=2:1:no_of_joints+1
    for i=2:1:no_of_joints+1
        HS_qk(:,:,k-1) = HS_qk(:,:,k-1) + J_I_i_qk(:,:,i-1,k-1) ...
                                        - mass_info(i)*vector2skew(J_T_Li_qk(1:3,4,i,k-1))*vector2skew(J_r_0i(:,i)) ...
                                        - mass_info(i)*vector2skew(J_r_0i(:,i))*vector2skew(J_T_Li_qk(1:3,4,i,k-1));
    end
end

for k=2:1:no_of_joints+1
    H0_qk(:,:,k-1) = [zeros(3,3), -m_tot*vector2skew(r0C_qk(:,k-1)); m_tot*vector2skew(r0C_qk(:,k-1)), HS_qk(:,:,k-1)];
end

for k=2:1:no_of_joints+1
    H_star_qk(:,:,k-1) = Hm_qk(:,:,k-1) ...
                        -H0m_qk(:,:,k-1)'*(H0\H0m) ...
                        -H0m'*(H0\H0m_qk(:,:,k-1)) ...
                        +H0m'*(H0\H0_qk(:,:,k-1))*(H0\H0m);
end

for k=2:1:no_of_joints+1
    c_star(k-1) = -0.5*q_dot'*H_star_qk(:,:,k-1)*q_dot;
end

C_star = H_star_dot*q_dot + c_star;

euler_to_bodyrate_matrix = [1, 0, -sin(J_T_J0_info(2));
                            0, cos(J_T_J0_info(1)), cos(J_T_J0_info(2))*sin(J_T_J0_info(1));
                            0, -sin(J_T_J0_info(1)), cos(J_T_J0_info(1))*cos(J_T_J0_info(2))];
% euler_to_bodyrate_matrix

euler_dot = euler_to_bodyrate_matrix\J_w_0;

for i=1:1:length(euler_dot)
    if abs(euler_dot(i))<threshold
        euler_dot(i)=0;
    end
end

if time==0
    euler=(J_T_J0_info(1:3))';
elseif time==time_step
        euler=((time_step)/6) * (euler_dot_prev1 + 2*(euler_dot_prev1+euler_dot) + euler_dot);
else
    euler=simpson((time-2*time_step),time,euler_dot_prev2,euler_dot_prev1,euler_dot)+euler_prev2;
end

euler_dot_prev2=euler_dot_prev1;
euler_dot_prev1=euler_dot;
euler_prev2=euler_prev1;
euler_prev1=euler;

J_T_J0_info(1:3)=euler';

J_T_J0 = JTJ0([euler', (J_x0(1:3))']);

joint_info(:,1) = [J_x0(5); q];
link_info(:,1) = [J_x0(5); q];

fprintf(fileID_2,'%12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f\n', ...
        J_x0(1), J_x0(2), J_x0(3), euler(1), euler(2), euler(3), ...
        J_x0_dot(1), J_x0_dot(2), J_x0_dot(3), euler_dot(1), euler_dot(2), euler_dot(3));

end
fclose(fileID);

%%%% PLOT  %%%%%

fileID_r = fopen(filename,'r');
fileID_r1 = fopen(filename_1,'r');
fileID_r2 = fopen(filename_2,'r');
fileID_r3 = fopen(filename_3,'r');
formatSpec = '%f %f %f %f';
sizeA = [4 Inf];

formatSpec_2 = '%f %f %f %f %f %f %f %f %f %f %f %f';
sizeA_2 = [12 Inf];

formatSpec_3 = '%f %f %f';
sizeA_3 = [3 Inf];

A = fscanf(fileID_r,formatSpec,sizeA);
A1 = fscanf(fileID_r1,formatSpec,sizeA);
A2 = fscanf(fileID_r2,formatSpec_2,sizeA_2);
A3 = fscanf(fileID_r3,formatSpec_3,sizeA_3);

fclose(fileID_r);
fclose(fileID_r1);
fclose(fileID_r2);
fclose(fileID_r3);

time = 0:time_step:end_time;

figure(1)
plot(time,A(1,:)); hold on; grid on;
plot(time,A(2,:)); 
plot(time,A(3,:)); 
plot(time,A(4,:)); 
xlabel('Time(s)');
ylabel('Joint Angles(rad)');
legend('Joint 1','Joint 2','Joint 3','Joint 4');
title('Joint Angles');

figure(2)
plot(time,A1(1,:)); hold on;  grid on;
plot(time,A1(2,:));
plot(time,A1(3,:)); 
plot(time,A1(4,:)); 
xlabel('Time(s)');
ylabel('Joint Angle Rates(rad/s)');
legend('Joint 1','Joint 2','Joint 3','Joint 4');
title('Joint Angle Rates');


figure(3)
plot(time,A2(1,:));  hold on;  grid on;
plot(time,A2(2,:));  
plot(time,A2(3,:));  
xlabel('Time(s)');
ylabel('Position(m)');
legend('Base x','Base y','Base z');
title('Base Position');

figure(4)
plot(time,A2(4,:)); hold on;  grid on;
plot(time,A2(5,:));  
plot(time,A2(6,:));  
xlabel('Time(s)');
ylabel('Angles(rad)');
legend('Base roll','Base pitch','Base yaw');
title('Base Euler Angles');

figure(5)
plot(time,A2(7,:));  hold on;  grid on;
plot(time,A2(8,:));    
plot(time,A2(9,:));  
xlabel('Time(s)');
ylabel('Velocity(m/s)');
legend('Base x','Base y','Base z');
title('Base Velocity');

figure(6)
plot(time,A2(10,:));  hold on;  grid on;
plot(time,A2(11,:)); 
plot(time,A2(12,:)); 
xlabel('Time(s)');
ylabel('Angular velocity(rad/s)');
legend('Base roll','Base pitch','Base yaw');
title('Base Euler Rates');

figure(7)
plot(time,A3(1,:));  hold on;  grid on;
plot(time,A3(2,:));  
plot(time,A3(3,:));  
xlabel('Time(s)');
ylabel('Position(m)');
legend('System x','System y','System z');
title('System COM');




function A = Ji_A_Jiplus1(jinfo) % input is a [1 X 4] matrix; output is a [4 X 4] matrix;
     A = [cos(jinfo(1)), -sin(jinfo(1))*cos(jinfo(3)), sin(jinfo(1))*sin(jinfo(3)), jinfo(4)*cos(jinfo(1)); 
          sin(jinfo(1)), cos(jinfo(1))*cos(jinfo(3)), -cos(jinfo(1))*sin(jinfo(3)), jinfo(4)*sin(jinfo(1));
          0, sin(jinfo(3)), cos(jinfo(3)), jinfo(2);
          0, 0, 0, 1];
end

function B = JTJ0(eulerxyz) % input is a [1 X 6] matrix; output is a [4 X 4] matrix;
roll=[1,0,0;
      0,cos(eulerxyz(1)),-sin(eulerxyz(1));
      0,sin(eulerxyz(1)),cos(eulerxyz(1))]; 
pitch=[cos(eulerxyz(2)),0,sin(eulerxyz(2));
       0,1,0;
       -sin(eulerxyz(2)),0,cos(eulerxyz(2))];
yaw=[cos(eulerxyz(3)),-sin(eulerxyz(3)),0;
     sin(eulerxyz(3)),cos(eulerxyz(3)),0;
     0,0,1];
R=yaw*pitch*roll; % dim(R) = [3 X 3]
B(1:3,1:3)=R;
B(1:3,4)=eulerxyz(4:6);
B(4,1:4)=[0,0,0,1];
end

function C = vector2skew(vector) % input is a [3 X 1] matrix; output is a [3 X 3] matrix;
C = [0,-vector(3),vector(2);vector(3),0,-vector(1);-vector(2),vector(1),0];
end

function y = simpson(a,b,y_dot_prev2,y_dot_prev1,y_dot)
y = ((b-a)/6) * (y_dot_prev2 + 4*y_dot_prev1 + y_dot);
end

function D = A_qk(jinfo) % input is a [1 X 4] matrix; output is a [4 X 4] matrix;
     D = [-sin(jinfo(1)), -cos(jinfo(1))*cos(jinfo(3)), cos(jinfo(1))*sin(jinfo(3)), -jinfo(4)*sin(jinfo(1)); 
          cos(jinfo(1)), -sin(jinfo(1))*cos(jinfo(3)), sin(jinfo(1))*sin(jinfo(3)), jinfo(4)*cos(jinfo(1));
          0, 0, 0, 0;
          0, 0, 0, 0];
end

