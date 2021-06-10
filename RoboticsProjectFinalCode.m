%% Robotics project
% This project is based on a 3 degree of freedom planar robot.
clear;
clc;

% Defining the robot link & setting link lenghts.
L1 = Link('revolute', 'd', 0, 'a', 9, 'alpha', 0);
L2 = Link('revolute', 'd', 0, 'a', 6, 'alpha', 0);
L3 = Link('revolute', 'd', 0, 'a', 1.5, 'alpha', 0);

%  Robot model object establishment
Hasbot=SerialLink([L1 L2 L3]);
Hasbot.name = 'Robot 1716197';

%Workspace initialization
Itcomp=zeros(4,4);
xlim([-10,10])
ylim([-5,10])
zlim([0,5])

% Process time
t=0:0.5:0.5;

% Robot is tasked to write/engrave 'Hasan' 
% using 3 Joint variable movement with one being the base rotation.
 
%% H

H0=[1.4984	2.0324	-1.9599];
H1=[1.3756	2.0583	-1.8631];
H2=[1.3146	2.0980	-1.8418];
H3=[1.3261	2.1433	-1.8986];
H4=[1.5682	2.2379	-2.2353];
H5=[1.5442	2.1397	-2.1131];
H6=[1.4075	2.2348	-2.0715];
H7=[1.3180	2.2930	-2.0402];
H8=[1.4421	2.3276	-2.1989];
H9=[1.4373	2.2906	-2.1571];
H10=[1.2292	2.1616	-1.8201];


% Letter H trajectory

Htraj1=jtraj(H0,H1,t); 
Htraj2=jtraj(H1,H2,t); 
Htraj3=jtraj(H2,H3,t); 
Htraj4=jtraj(H3,H4,t);
Htraj5=jtraj(H4,H5,t);
Htraj6=jtraj(H5,H6,t);
Htraj7=jtraj(H6,H7,t);
Htraj8=jtraj(H7,H8,t);
Htraj9=jtraj(H8,H9,t);
Htraj10=jtraj(H9,H10,t);

hold on

%  Segment 1 trajectory
for i=1:1:length(t)
    Hasb=Hasbot.fkine(Htraj1(i,:));
    H_letter(i,:)=transl(Hasb);  %#ok<*SAGROW> %  Extract the translation component of the pose 
                           %  matrix (3Element column vector) stored in JTA 
                           %  vector array
    jta=H_letter; 
    plot2(jta(i,:),'r.')   %  Track points (red points) Sketch
      Hasbot.plot(Htraj1(i,:),'scale',0.3,'linkcolor','k','jointcolor','c') %  Drawing trajectory animation
    plot2(H_letter,'m','linewidth',2)         %  Draw a trajectory line (magenta line)
end
%  Segment 2 trajectory
for i=1:1:length(t)
    Hasb2=Hasbot.fkine(Htraj2(i,:));
    H_letter2(i,:)=transl(Hasb2);  
    
    jta2=H_letter2; 
    plot2(jta2(i,:),'r.')   
    Hasbot.plot(Htraj2(i,:)) 
    plot2(H_letter2,'m','linewidth',2)         
end
%  Segment 3 trajectory
for i=1:1:length(t)
    Hasb3=Hasbot.fkine(Htraj3(i,:));
    H_letter3(i,:)=transl(Hasb3);  
    
    jta3=H_letter3; 
    plot2(jta3(i,:),'r.')   
    Hasbot.plot(Htraj3(i,:)) 
    plot2(H_letter3,'m','linewidth',2)        
end
%  Segment 4 trajectory
for i=1:1:length(t)
    Hasb4=Hasbot.fkine(Htraj4(i,:));
    H_letter4(i,:)=transl(Hasb4);  
    
    jta4=H_letter4; 
    plot2(jta4(i,:),'r.')   
    Hasbot.plot(Htraj4(i,:)) 
    plot2(H_letter4,'m','linewidth',2)        
end
%  Segment 5 trajectory
for i=1:1:length(t)
    Hasb5=Hasbot.fkine(Htraj5(i,:));
    H_letter5(i,:)=transl(Hasb5);  
    
%     jta5=H_letter5; 
%     plot2(jta5(i,:),'r.')   
%     Hasbot.plot(Htraj5(i,:)) 
%     plot2(H_letter5,'m','linewidth',2)         
end
%  Segment 6 trajectory
for i=1:1:length(t)
    Hasb6=Hasbot.fkine(Htraj6(i,:));
    H_letter6(i,:)=transl(Hasb6);  
    
    jta6=H_letter6; 
    plot2(jta6(i,:),'r.')   
    Hasbot.plot(Htraj6(i,:)) 
    plot2(H_letter6,'m','linewidth',2)         
end
%  Segment 7 trajectory
for i=1:1:length(t)
    Hasb7=Hasbot.fkine(Htraj7(i,:));
    H_letter7(i,:)=transl(Hasb7);  
    
    jta7=H_letter7; 
    plot2(jta7(i,:),'r.')   
    Hasbot.plot(Htraj7(i,:)) 
    plot2(H_letter7,'m','linewidth',2)         
end
%  Segment 8 trajectory
for i=1:1:length(t)
    Hasb8=Hasbot.fkine(Htraj8(i,:));
    H_letter8(i,:)=transl(Hasb8);  
    
%     jta8=H_letter8; 
%     plot2(jta8(i,:),'r.')   
     Hasbot.plot(Htraj8(i,:)) 
%     plot2(H_letter8,'m','linewidth',2)         
end
%  Segment 9 trajectory
for i=1:1:length(t)
    Hasb9=Hasbot.fkine(Htraj9(i,:));
    H_letter9(i,:)=transl(Hasb9);  
    
    jta9=H_letter9; 
    plot2(jta9(i,:),'r.')   
    Hasbot.plot(Htraj9(i,:)) 
    plot2(H_letter9,'m','linewidth',2)         
end
%  Segment 10 trajectory
for i=1:1:length(t)
    Hasb10=Hasbot.fkine(Htraj10(i,:));
    H_letter10(i,:)=transl(Hasb10);  
    jta10=H_letter10; 
    plot2(jta10(i,:),'r.')   
    Hasbot.plot(Htraj10(i,:)) 
    plot2(H_letter10,'m','linewidth',2)         
end

%% A


Ao0=[1.3846	2.4098	-2.2236];
Ao1=[0.9550	2.2111	-1.5953];
Ao2=[1.1552	2.4973	-2.0817];
Ao3=[1.0826	2.4612	-1.9730];
Ao4=[1.2578	2.3068	-1.9937];
Ao5=[1.0230	2.3856	-1.8378];



% letter A trajectory
Aotraj1=jtraj(Ao0,Ao1,t); 
Aotraj2=jtraj(Ao1,Ao2,t); 
Aotraj3=jtraj(Ao3,Ao4,t); 
Aotraj4=jtraj(Ao4,Ao5,t);
Aotraj5=jtraj(Ao4,Ao5,t);
hold on

%  Segment 1 trajectory
for i=1:1:length(t)
    Hasb=Hasbot.fkine(Aotraj1(i,:));
    Ao_letter(i,:)=transl(Hasb);  
    
    jta=Ao_letter; 
    plot2(jta(i,:),'r.')   
    Hasbot.plot(Aotraj1(i,:),'scale',0.3,'linkcolor','k','jointcolor','c') 
    plot2(Ao_letter,'m','linewidth',2)         
end
%  Segment 2 trajectory
for i=1:1:length(t)
    Hasb2=Hasbot.fkine(Aotraj2(i,:));
    Ao_letter2(i,:)=transl(Hasb2);  
    
    jta2=Ao_letter2; 
    plot2(jta2(i,:),'r.')  
    Hasbot.plot(Aotraj2(i,:)) 
    plot2(Ao_letter2,'m','linewidth',2)       
end
%  Segment 3 trajectory
for i=1:1:length(t)
    Hasb3=Hasbot.fkine(Aotraj3(i,:));
    Ao_letter3(i,:)=transl(Hasb3);  
    
    %jta3=Ao_letter3; 
    %plot2(jta3(i,:),'r.')   
    Hasbot.plot(Aotraj3(i,:)) 
    %plot2(Ao_letter3,'m','linewidth',2)         
end
%  Segment 4 trajectory
for i=1:1:length(t)
    Hasb4=Hasbot.fkine(Aotraj4(i,:));
    Ao_letter4(i,:)=transl(Hasb4);  
    
    jta4=Ao_letter4; 
    plot2(jta4(i,:),'r.')   
    Hasbot.plot(Aotraj4(i,:)) 
    plot2(Ao_letter4,'m','linewidth',2)         
end
%   Segment 5 trajectory
for i=1:1:length(t)
    Hasb5=Hasbot.fkine(Aotraj5(i,:));
    Ao_letter5(i,:)=transl(Hasb5);  
    
    %jta5=Ao_letter5; 
    %plot2(jta5(i,:),'r.')   
    Hasbot.plot(Aotraj5(i,:)) 
    %plot2(Ao_letter5,'m','linewidth',2)         
end

%% S

S0=[1.0469	2.5049	-1.9810];
S1=[0.8728	2.5379	-1.8399];
S2=[0.7491	2.4473	-1.6256];
S3=[0.9379	2.3639	-1.7310];
S4=[0.8415	2.2686	-1.5393];
S5=[0.6980	2.2565	-1.3837];
S6=[0.7648	2.3320	-1.5260];

% Letter S trajectory
Straj1=jtraj(S0,S1,t); 
Straj2=jtraj(S1,S2,t); 
Straj3=jtraj(S2,S3,t); 
Straj4=jtraj(S3,S4,t);
Straj5=jtraj(S4,S5,t);
Straj6=jtraj(S5,S6,t);

hold on

%  Segment 1 trajectory
for i=1:1:length(t)
    Hasb=Hasbot.fkine(Straj1(i,:));
    S_letter(i,:)=transl(Hasb);  
    
    jta=S_letter; 
    plot2(jta(i,:),'r.')   
    Hasbot.plot(Straj1(i,:),'scale',0.3,'linkcolor','k','jointcolor','c') 
    plot2(S_letter,'m','linewidth',2)         
end
%  Segment 2 trajectory
for i=1:1:length(t)
    Hasb2=Hasbot.fkine(Straj2(i,:));
    S_letter2(i,:)=transl(Hasb2);  
    
    jta2=S_letter2; 
    plot2(jta2(i,:),'r.')   
    Hasbot.plot(Straj2(i,:)) 
    plot2(S_letter2,'m','linewidth',2)         
end
%  Segment 3 trajectory
for i=1:1:length(t)
    Hasb3=Hasbot.fkine(Straj3(i,:));
    S_letter3(i,:)=transl(Hasb3);  
    
    jta3=S_letter3; 
    plot2(jta3(i,:),'r.')   
    Hasbot.plot(Straj3(i,:)) 
    plot2(S_letter3,'m','linewidth',2)         
end
%  Segment 4 trajectory
for i=1:1:length(t)
    Hasb4=Hasbot.fkine(Straj4(i,:));
    S_letter4(i,:)=transl(Hasb4);  
    
    jta4=S_letter4; 
    plot2(jta4(i,:),'r.')   
    Hasbot.plot(Straj4(i,:))
    plot2(S_letter4,'m','linewidth',2)        
end
%  Segment 5 trajectory
for i=1:1:length(t)
    Hasb5=Hasbot.fkine(Straj5(i,:));
    S_letter5(i,:)=transl(Hasb5);  
    
    jta5=S_letter5; 
    plot2(jta5(i,:),'r.')   
    Hasbot.plot(Straj5(i,:)) 
    plot2(S_letter5,'m','linewidth',2)        
end
% %  Segment 6 trajectory
for i=1:1:length(t)
    Hasb5=Hasbot.fkine(Straj5(i,:));
    S_letter6(i,:)=transl(Hasb5);  
    
    jta5=S_letter6; 
    plot2(jta5(i,:),'r.')   
    Hasbot.plot(Straj6(i,:)) 
    plot2(S_letter6,'m','linewidth',2)         
end

%% A

A0=[0.7222	2.5446	-1.6960];
A1=[0.4591	2.1176	-1.0060];
A2=[0.4653	2.4473	-1.3418];
A3=[0.4103	2.3897	-1.2291];
A4=[0.4244	2.3068	-1.1603];
A5=[0.6362	2.3628	-1.4282];

% Letter A trajectory
t=0:0.33:1;
Atraj1=jtraj(A0,A1,t); 
Atraj2=jtraj(A1,A2,t); 
Atraj3=jtraj(A3,A4,t); 
Atraj4=jtraj(A4,A5,t);
Atraj5=jtraj(A4,A5,t);
hold on

%  Segment 1 trajectory
for i=1:1:length(t)
    Hasb=Hasbot.fkine(Atraj1(i,:));
    A_letter(i,:)=transl(Hasb); 
    
    jta=A_letter; 
    plot2(jta(i,:),'r.')   
    Hasbot.plot(Atraj1(i,:),'scale',0.3,'linkcolor','k','jointcolor','c') 
    plot2(A_letter,'m','linewidth',2)        
end
%  Segment 2 trajectory
for i=1:1:length(t)
    Hasb2=Hasbot.fkine(Atraj2(i,:));
    A_letter2(i,:)=transl(Hasb2);  
    
    jta2=A_letter2; 
    plot2(jta2(i,:),'r.')   
    Hasbot.plot(Atraj2(i,:)) 
    plot2(A_letter2,'m','linewidth',2)        
end
%  Segment 3 trajectory
for i=1:1:length(t)
    Hasb3=Hasbot.fkine(Atraj3(i,:));
    A_letter3(i,:)=transl(Hasb3); 
    
    %jta3=A_letter3; 
    %plot2(jta3(i,:),'r.')   
    Hasbot.plot(Atraj3(i,:)) 
    %plot2(A_letter3,'m','linewidth',2)         
end
%  Segment 4 trajectory
for i=1:1:length(t)
    Hasb4=Hasbot.fkine(Atraj4(i,:));
    A_letter4(i,:)=transl(Hasb4);  
    
    jta4=A_letter4; 
    plot2(jta4(i,:),'r.')   
    Hasbot.plot(Atraj4(i,:)) 
    plot2(A_letter4,'m','linewidth',2)        
end
%  Segment 5 trajectory
for i=1:1:length(t)
    Hasb5=Hasbot.fkine(Atraj5(i,:));
    A_letter5(i,:)=transl(Hasb5);  
    
    %jta5=A_letter5; 
    %plot2(jta5(i,:),'r.')   
    Hasbot.plot(Atraj5(i,:))
    %plot2(A_letter5,'m','linewidth',2)         
end

%% N


N0=[0.4046	2.1432	-0.9771];
N1=[0.3672	2.0516	-0.8480];
N2=[0.3230	2.3809	-1.1331];
N3=[0.2889	1.9955	-0.7135];
N4=[0.2109	2.3452	-0.9853];
N5=[0.1664	2.1976	-0.7932];


% Trajectory planning for the letter N

Ntraj1=jtraj(N0,N1,t); 
Ntraj2=jtraj(N1,N2,t); 
Ntraj3=jtraj(N2,N3,t); 
Ntraj4=jtraj(N3,N4,t);
Ntraj5=jtraj(N4,N5,t);


hold on

%  Segment 1 trajectory
for i=1:1:length(t)
    Hasb=Hasbot.fkine(Ntraj1(i,:));
    N_letter(i,:)=transl(Hasb);  
    
    jta=N_letter; 
    plot2(jta(i,:),'r.')  
    Hasbot.plot(Ntraj1(i,:),'scale',0.3,'linkcolor','k','jointcolor','c') 
    plot2(N_letter,'m','linewidth',2)         
end
%  Segment 2 trajectory
for i=1:1:length(t)
    Hasb2=Hasbot.fkine(Ntraj2(i,:));
    N_letter2(i,:)=transl(Hasb2); 
    
    jta2=N_letter2; 
    plot2(jta2(i,:),'r.')   
    Hasbot.plot(Ntraj2(i,:)) 
    plot2(N_letter2,'m','linewidth',2)         
end
%  Segment 3 trajectory
for i=1:1:length(t)
    Hasb3=Hasbot.fkine(Ntraj3(i,:));
    N_letter3(i,:)=transl(Hasb3); 
                          
    jta3=N_letter3; 
    plot2(jta3(i,:),'r.')   
    Hasbot.plot(Ntraj3(i,:)) 
    plot2(N_letter3,'m','linewidth',2)         
end
%  Segment 4 trajectory
for i=1:1:length(t)
    Hasb4=Hasbot.fkine(Ntraj4(i,:));
    N_letter4(i,:)=transl(Hasb4);  
    
    jta4=N_letter4; 
    plot2(jta4(i,:),'r.')   
    Hasbot.plot(Ntraj4(i,:)) 
    plot2(N_letter4,'m','linewidth',2)         
end
% %  Segment 5 trajectory
for i=1:1:length(t)
    Hasb5=Hasbot.fkine(Ntraj5(i,:));
    N_letter5(i,:)=transl(Hasb5);  
    
    jta5=N_letter5; 
    plot2(jta5(i,:),'r.')   
    Hasbot.plot(Ntraj5(i,:)) 
    plot2(N_letter5,'m','linewidth',2)         
end


