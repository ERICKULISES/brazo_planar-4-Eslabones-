%% Limpia la memoria de variables
clear all
close all
clc

%% Cierra y elimina cualquier objeto de tipo serial 
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%% Creación de un objeto tipo serial
arduino = serial('COM16','BaudRate',9600);
fopen(arduino);
if arduino.status == 'open'
    disp('Arduino conectado correctamente \n');
else
    disp('No se ha conectado el arduino \n');
    return
end

%%
prompt = 'Introducir el valor L1:';
L1 = input (prompt);
prompt = 'Introducir el valor L2:';
L2 = input (prompt);
prompt = 'Introducir el valor L3:';
L3 = input (prompt);
%% Se establece el número de muestras y el contador para poder utilizarlos en el blucle principal 
numero_muestras = 1000;
y = zeros(1,numero_muestras); 
contador_muestras = 1; 
figure('Name','Brazo Robótico 3 eslabones + Arduino. TESE-Robótica')
title('Comunicación Serial MATLAB + ARDUINO');
xlabel('Número de muestra');
ylabel('Valor');
grid on;
hold on;

valor_con_offset = fscanf(arduino,'%d,%d,%d´');
theta1_deg = ((valor_con_offset(1))-512)*130/512;
theta2_deg = ((valor_con_offset(2))-512)*130/512;
theta3_deg = ((valor_con_offset(3))-512)*130/512;

theta1_rad = deg2rad(theta1_deg);
theta2_rad = deg2rad(theta2_deg);
theta3_rad = deg2rad(theta3_deg);

d1 = L1;
d2 = 0;
d3 = 0;

a1 = 0;
a2 = L2;
a3 = L3;
alpha_1 = 90;
alpha_2 = 0;
alpha_3 = 0;
alpha_1_rad = deg2rad(alpha_1);
alpha_2_rad = deg2rad(alpha_2);
alpha_3_rad = deg2rad(alpha_3);
p1 =[0 0 0];
if theta1_rad>=0
    angVec = 0:0.01:theta1_rad;
else
    angVec = 0:-0.01:theta1_rad;
end
for i=1:length(angVec)
    clf
    printAxis();
    grid on 
    Rotz = [cos(angVec(i)) -sin(angVec(i))  0; sin(angVec(i)) cos(angVec(i)) 0; 0 0 1];
    A1 = dhParameters(angVec(i),d1,a1,alpha_1_rad);
    A2 = dhParameters(0,d2,a2,alpha_2_rad);
    A3 = dhParameters(0,d3,a3,alpha_3_rad);
    A12 = A1*A2;
    A123 = A1*A2*A3; 
    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    grid on
    pause(0.01);
end
pause(1);
    
if theta2_rad>=0
    angVec = 0:0.01:theta2_rad;
else
    angVec = 0:-0.01:theta2_rad;
end
for i=1:length(angVec)
    clf
    printAxis();
    grid on 
    Rotz = [cos(theta1_rad) -sin(theta1_rad)  0; sin(theta1_rad) cos(theta1_rad) 0; 0 0 1];
    A1 = dhParameters(theta1_rad,d1,a1,alpha_1_rad);
    A2 = dhParameters(angVec(i),d2,a2,alpha_2_rad);
    A3 = dhParameters(0,d3,a3,alpha_3_rad);
    A12 = A1*A2;
    A123 = A1*A2*A3; 
    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    pause(0.01);
end
pause(1);

if theta3_rad>=0
    angVec = 0:0.01:theta3_rad;
else
    angVec = 0:-0.01:theta3_rad;
end

for i=1:length(angVec)
    clf
    printAxis();
    grid on 
    Rotz = [cos(theta1_rad) -sin(theta1_rad)  0; sin(theta1_rad) cos(theta1_rad) 0; 0 0 1];
    A1 = dhParameters(theta1_rad,d1,a1,alpha_1_rad);
    A2 = dhParameters(theta2_rad,d2,a2,alpha_2_rad);
    A3 = dhParameters(angVec(i),d3,a3,alpha_3_rad);
    A12 = A1*A2;
    A123 = A1*A2*A3; 
    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    pause(0.01);
end


%% Cierre de puertos
fclose(arduino);
delete(arduino);
clear all; 