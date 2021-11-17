%% Clear workspace

clear
%clc

%% Equation parameters

data_base_length = 1000;
variables = 8;
variables = variables + 3;

k_data_base = zeros(data_base_length,variables);

for j = 1:data_base_length

C_m_delta_e = -1.3265;

U_0 = linspace(1,100,500); %m/s
U_0 = U_0(randi(500));

rho = 1.29;

st = linspace(0,1,500);
st = st(randi(500));

c = linspace(0.05,0.5,500);
c = c(randi(500));

I_y = 0.17;


C_z_delta_e = -3.565;


m = linspace(1000,10000,500);
m = m(randi(500));

C_mq = 1;
C_m_alpha = 1;
C_m_alpha_dot = 1;
C_l_alpha = 1;
C_do = 0;


Q_0 = (rho * U_0^2)/2;
M_q = C_mq * c * Q_0 * st * ((c)/(2 * U_0 * I_y));
M_wd = C_m_alpha_dot * (((c)/(2 * U_0)) * Q_0 * st * ((c)/(U_0 * I_y)));
M_w = C_m_alpha * (Q_0 * st * ((c)/(U_0 * I_y)));
Z_w = (-(C_l_alpha + C_do) * Q_0 * ((st)/(m * U_0)));
M_delta_e = C_m_delta_e * Q_0 * st * (c/I_y);
M_alpha_dot = U_0*M_wd;
Z_delta_e = C_z_delta_e * Q_0 * (st/m);
M_alpha = U_0 * M_w;
Z_alpha = U_0 * Z_w;


a_s = M_delta_e + ((M_alpha_dot*Z_delta_e)/(U_0));
b_s = ((M_alpha*Z_delta_e)/(U_0)) - ((M_delta_e*Z_alpha)/(U_0));
c_s = M_q + M_alpha_dot + ((Z_alpha)/(U_0));
d_s = ((Z_alpha*M_q)/(U_0)) - M_alpha;

%% Control definition

s = tf('s');
tf_pitch = ((-a_s*s - b_s)/(s^3 - c_s * s^2 + d_s * s));

% figure (1)
% step(tf_pitch,10)
% tf_info = stepinfo(tf_pitch);
% 
% figure(2)
% step(tf_pitch,0.1)
% 
% figure(3)
% pzmap(tf_pitch)
% 
% isstable(tf_pitch)

%% Control application for Kd

kd = 0.01; % variable inicial de Kd seleccionada
kd_val = zeros(1,10);
kd_val_fine = zeros(1,100);
kd_stable_corse = zeros(1,length(kd_val));
kd_info_corse = zeros(1,length(kd_val));
kd_stable_medium = zeros(1,length(kd_val));
kd_info_medium = zeros(1,length(kd_val));
kd_stable_fine = zeros(1,length(kd_val_fine));
kd_info_fine = zeros(1,length(kd_val_fine));

for i = 1:length(kd_val)    
    C(:,:,i) = pid(1,1,kd);
    kd_val(i) = kd;
    kd = kd + 1;    
end

for i = 1:length(kd_val)
    pitch_kd_corse = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_corse)     
    kd_stable_corse(i) = isstable(pitch_kd_corse);    
    info_corse = stepinfo(pitch_kd_corse);   
    kd_info_corse(i) = info_corse.SettlingTime;
end


if max(kd_stable_corse) == 1
   kd_best_corse = min(kd_info_corse);
   kd_best_corse_index = find(kd_info_corse == kd_best_corse);
   kd = kd_val(kd_best_corse_index); 
   if length(kd) > 1
      kd = kd(1);
   end  
else
    kd = 1;
end

for i = 1:length(kd_val)    
    C(:,:,i) = pid(1,1,kd);
    kd_val(i) = kd;
    kd = kd + 0.5;    
end

for i = 1:length(kd_val)
    pitch_kd_medium = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_medium)     
    kd_stable_medium(i) = isstable(pitch_kd_medium);    
    info_medium = stepinfo(pitch_kd_medium);   
    kd_info_medium(i) = info_medium.SettlingTime;
end


if max(kd_stable_medium) == 1
   kd_best_medium = min(kd_info_medium);
   kd_best_medium_index = find(kd_info_medium == kd_best_medium);
   kd = kd_val(kd_best_medium_index); 
   if length(kd) > 1
      kd = kd(1);
   end  
else
    kd = 0.1;
end

for i = 1:length(kd_val_fine)    
    C(:,:,i) = pid(1,1,kd);
    kd_val_fine(i) = kd;
    kd = kd + 0.05;    
end

for i = 1:length(kd_val_fine)
    pitch_kd_fine = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_medium)     
    kd_stable_fine(i) = isstable(pitch_kd_fine);    
    info_fine = stepinfo(pitch_kd_fine);   
    kd_info_fine(i) = info_fine.SettlingTime;
end


if max(kd_stable_fine) == 1
   kd_best_fine = min(kd_info_fine);
   kd_best_fine_index = find(kd_info_fine == kd_best_fine);
   kd = kd_val_fine(kd_best_fine_index); 
   if length(kd) > 1
      kd = kd(1);
   end  
else
    kd = 0.1;
end

%% Control aplication for Ki


ki = 0.01; % variable inicial de Kd seleccionada
ki_val = zeros(1,10);
ki_val_fine = zeros(1,100);
ki_stable_corse = zeros(1,length(ki_val));
ki_info_corse = zeros(1,length(ki_val));
ki_stable_medium = zeros(1,length(ki_val));
ki_info_medium = zeros(1,length(ki_val));
ki_stable_fine = zeros(1,length(ki_val_fine));
ki_info_fine = zeros(1,length(ki_val_fine));

for i = 1:length(ki_val)    
    C(:,:,i) = pid(1,ki,kd);
    ki_val(i) = ki;
    ki = ki + 1;    
end

for i = 1:length(ki_val)
    pitch_ki_corse = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_corse)     
    ki_stable_corse(i) = isstable(pitch_ki_corse);    
    info_corse = stepinfo(pitch_ki_corse);   
    ki_info_corse(i) = info_corse.SettlingTime;
end


if max(ki_stable_corse) == 1
   ki_best_corse = min(ki_info_corse);
   ki_best_corse_index = find(ki_info_corse == ki_best_corse);
   ki = ki_val(ki_best_corse_index); 
   if length(ki) > 1
      ki = ki(1);
   end  
else
    ki = 1;
end

for i = 1:length(ki_val)    
    C(:,:,i) = pid(1,ki,kd);
    ki_val(i) = ki;
    ki = ki + 0.5;    
end

for i = 1:length(ki_val)
    pitch_ki_medium = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_medium)     
    ki_stable_medium(i) = isstable(pitch_ki_medium);    
    info_medium = stepinfo(pitch_ki_medium);   
    ki_info_medium(i) = info_medium.SettlingTime;
end


if max(ki_stable_medium) == 1
   ki_best_medium = min(ki_info_medium);
   ki_best_medium_index = find(ki_info_medium == ki_best_medium);
   ki = ki_val(ki_best_medium_index); 
   if length(ki) > 1
      ki = ki(1);
   end  
else
    ki = 0.1;
end

for i = 1:length(ki_val_fine)    
    C(:,:,i) = pid(1,ki,kd);
    ki_val_fine(i) = ki;
    ki = ki + 0.05;    
end

for i = 1:length(ki_val_fine)
    pitch_ki_fine = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_medium)     
    ki_stable_fine(i) = isstable(pitch_ki_fine);    
    info_fine = stepinfo(pitch_ki_fine);   
    ki_info_fine(i) = info_fine.SettlingTime;
end


if max(ki_stable_fine) == 1
   ki_best_fine = min(ki_info_fine);
   ki_best_fine_index = find(ki_info_fine == ki_best_fine);
   ki = ki_val_fine(ki_best_fine_index); 
   if length(ki) > 1
      ki = ki(1);
   end 
else
    ki = 0.1;
end

%% Control for Kp

kp = 0.01; % variable inicial de Kd seleccionada
kp_val = zeros(1,10);
kp_val_fine = zeros(1,100);
kp_stable_corse = zeros(1,length(kp_val));
kp_info_corse = zeros(1,length(kp_val));
kp_stable_medium = zeros(1,length(kp_val));
kp_info_medium = zeros(1,length(kp_val));
kp_stable_fine = zeros(1,length(kp_val_fine));
kp_info_fine = zeros(1,length(kp_val_fine));

for i = 1:length(kp_val)    
    C(:,:,i) = pid(kp,ki,kd);
    kp_val(i) = kp;
    kp = kp + 1;    
end

for i = 1:length(kp_val)
    pitch_kp_corse = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_corse)     
    kp_stable_corse(i) = isstable(pitch_ki_corse);    
    info_corse = stepinfo(pitch_ki_corse);   
    kp_info_corse(i) = info_corse.SettlingTime;
end


if max(kp_stable_corse) == 1
   kp_best_corse = min(kp_info_corse);
   kp_best_corse_index = find(kp_info_corse == kp_best_corse);
   kp = kp_val(kp_best_corse_index); 
   if length(kp) > 1
       kp = kp(1);
   end
else
    kp = 1;
end

for i = 1:length(kp_val)    
    C(:,:,i) = pid(kp,ki,kd);
    kp_val(i) = kp;
    kp = kp + 0.5;    
end

for i = 1:length(kp_val)
    pitch_kp_medium = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_medium)     
    kp_stable_medium(i) = isstable(pitch_kp_medium);    
    info_medium = stepinfo(pitch_kp_medium);   
    kp_info_medium(i) = info_medium.SettlingTime;
end


if max(kp_stable_medium) == 1
   kp_best_medium = min(kp_info_medium);
   kp_best_medium_index = find(kp_info_medium == kp_best_medium);
   kp = kp_val(kp_best_medium_index); 
   if length(kp) > 1
      kp = kp(1);
   end
else
    kp = 0.1;
end

for i = 1:length(kp_val_fine)    
    C(:,:,i) = pid(kp,ki,kd);
    kp_val_fine(i) = kp;
    kp = kp + 0.05;    
end

for i = 1:length(kp_val_fine)
    pitch_kp_fine = feedback(C(:,:,i)*tf_pitch,1);  
%     figure(i+3)
%     step(pitch_kp_medium)     
    kp_stable_fine(i) = isstable(pitch_kp_fine);    
    info_fine = stepinfo(pitch_kp_fine);   
    kp_info_fine(i) = info_fine.SettlingTime;
end


if max(kp_stable_fine) == 1
   kp_best_fine = min(kp_info_fine);
   kp_best_fine_index = find(kp_info_fine == kp_best_fine);
   kp = kp_val_fine(kp_best_fine_index); 
   if length(kp) > 1
      kp = kp(1);
   end
else
    kp = 0.1;
end

k_data_base(j,:) = [C_m_delta_e U_0 rho st c I_y C_z_delta_e m kp ki kd]; 

end

%% Summary

k_data_base = array2table(k_data_base,'VariableNames',{'C_m_delta_e' 'U_0' 'rho' 'st' 'c' 'I_y' 'C_z_delta_e' 'm' 'kp' 'ki' 'kd'})

% K_control = [kp ki kd];
% K_control = array2table(K_control, 'VariableNames',{'K_p','K_i','K_d'})
% 
% C(:,:,i) = pid(kp,ki,kd);
% pitch_final = feedback(C(:,:,i)*tf_pitch,1);
% 
% figure(4)
% step(pitch_final)    
% 
% stable_finish = isstable(pitch_final);    
% info_final = stepinfo(pitch_final);   
% info_final = info_final.SettlingTime;


