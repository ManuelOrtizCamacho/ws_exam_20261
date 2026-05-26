clear all
clc
close all
%% Direct Kinematic Model

% DH Matrix

        DH=[]

frame=size(DH,1);


for i=1:frame
    T_ij(:,:,i)=T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
end

T(:,:,1)=T_ij(:,:,1); 

for i=1:frame-1
    T(:,:,i+1)=T(:,:,i)*T_ij(:,:,i+1);
end

T

%% Graph


hf=figure(1); 
set(hf,'position',[445   275   563   628])
L=10; 
h_ejexF=plot3([0 L],[0 0],[0 0],'r','linewidth',2.5); % dibujo los ejer cordenados plot3(x,y,z)
hold on 
h_ejeyF=plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
hold on
h_ejezF=plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
hold on

axis equal
axis([0 0 0 0 0 0]) 
xlabel('X [-]')
ylabel('Y [-]')
zlabel('Z [-]')
view(27,37) 


for i=1:frame
    h_ejexM(i,1)=plot3([0 0],[0 0],[0 0],'r','linewidth',2);
    hold on
    h_ejeyM(i,1)=plot3([0 0],[0 0],[0 0],'g','linewidth',2);
    hold on
    h_ejezM(i,1)=plot3([0 0],[0 0],[0 0],'b','linewidth',2);
end


Mp=[0,0,0,1;  % origen
    L,0,0,1;  % eje x
    0,L,0,1;  % eje y
    0,0,L,1]; % eje z

% Points Matrix on {0}
for k=1:frame
    for i=1:4
        Mp_T(i,:,k)=(T(:,:,k)*Mp(i,:)')';
    end
end

for k=1:frame
    set(h_ejexM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(2,1,k)],...
                     'ydata',[Mp_T(1,2,k) Mp_T(2,2,k)],...
                     'zdata',[Mp_T(1,3,k) Mp_T(2,3,k)])

    set(h_ejeyM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(3,1,k)],...
                     'ydata',[Mp_T(1,2,k) Mp_T(3,2,k)],...
                     'zdata',[Mp_T(1,3,k) Mp_T(3,3,k)])

    set(h_ejezM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(4,1,k)],...
                     'ydata',[Mp_T(1,2,k) Mp_T(4,2,k)],...
                     'zdata',[Mp_T(1,3,k) Mp_T(4,3,k)])
end

%% Transform tipo Denavit-Hartemberg
function T_ij=T_DH(a_ij,alpha_ij,s_i,theta_i)

T_ij =[               cos(theta_i),              -sin(theta_i),              0,               a_ij;
        cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
        sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
                          0,                          0,              0,                  1];
end
