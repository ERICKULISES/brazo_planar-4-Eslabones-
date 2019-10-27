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
    disp('Arduino conectado correctamente');
else
    disp('No se ha conectado el arduino');
    return
end
%% Configuración de las longitudes del brazo
prompt = 'Introducir el valor L1:';
L1 = input (prompt);
prompt = 'Introducir el valor L2:';
L2 = input (prompt);
prompt = 'Introducir el valor L3:';
L3 = input (prompt);
prompt = 'Introducir el valor L4:';
L4 = input (prompt);
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
%% Definición de los parametros de Denavit-Hartenberg
d1 = L1;
d2 = 0;
d3 = 0;
d4 = 0;
a1 = 0;
a2 = L2;
a3 = L3;
a4 = L4;
alpha_1 = 90;
alpha_2 = 0;
alpha_3 = 0;
alpha_4 = 0;
alpha_1_rad = deg2rad(alpha_1);
alpha_2_rad = deg2rad(alpha_2);
alpha_3_rad = deg2rad(alpha_3);
alpha_4_rad = deg2rad(alpha_4);
%% Definición del punto incial
p1 =[0 0 0];
while 1
    clf
    printAxis();
%% Obtiene los valores del Arduino y mediante la formula se tiene su dimecion de cada uno de sus  grados
    valor_con_offset = fscanf(arduino,'%d,%d,%d,%d´');
    theta1_deg = ((valor_con_offset(1))-512)*130/512;
    theta1_rad = deg2rad(theta1_deg);
    disp('Longitud del primer eslabon en grados:')
    disp(theta1_deg)
    theta2_deg = ((valor_con_offset(2))-512)*130/512;
    theta2_rad = deg2rad(theta2_deg);
    disp('Longitud del segundo eslabon en grados:')
    disp(theta2_deg)
    theta3_deg = ((valor_con_offset(3))-512)*130/512;
    theta3_rad = deg2rad(theta3_deg);
    disp('Longitud del tercer eslabon en grados:')
    disp(theta3_deg)
    theta4_deg = ((valor_con_offset(4))-512)*130/512;
    theta4_rad = deg2rad(theta4_deg);
    disp('Longitud del cuarto eslabon en grados:')
    disp(theta4_deg)
%% Desarrollo de las matrices
    Rotz = [cos(theta1_rad) -sin(theta1_rad) 0 0; sin(theta1_rad) cos(theta1_rad) 0 0; 0 0 1 0; 0 0 0 1];
    A1 = dhParameters(theta1_rad,d1,a1,alpha_1_rad);
    A2 = dhParameters(theta2_rad,d2,a2,alpha_2_rad);
    A3 = dhParameters(theta3_rad,d3,a3,alpha_3_rad);
    A4 = dhParameters(theta4_rad,d4,a4,alpha_4_rad);
    A12 = A1*A2;
    A123 = A1*A2*A3; 
    A1234 = A1*A2*A3*A4; 
%% Se indica los distintos puntos que tendrá el brazo
    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    p5 = A1234(1:3,4);
%% Configuración del grosor y color que tendrá cada eslabon del brazo
 line([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'color',[0 0 0],'linewidth',2.5)
 line([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)],'color',[0 0 0],'linewidth',2.5)
 line([p3(1) p4(1)],[p3(2) p4(2)],[p3(3) p4(3)],'color',[0 0 0],'linewidth',2.5)
 line([p4(1) p5(1)],[p4(2) p5(2)],[p4(3) p5(3)],'color',[0 0 0],'linewidth',2.5)

%% Configuración de los ejes de referencia que tendrá el brazo los cuales son los que nos indica en que eje se rotará
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A1234);
    printMiniAxes(p5,A1234);
    view(30,30);
    grid on
    pause(0.01);
 end
%% Cierre de puertos
fclose(arduino);
delete(arduino);
clear all; 

