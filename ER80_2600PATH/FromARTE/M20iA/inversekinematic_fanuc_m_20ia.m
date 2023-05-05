
function q = inversekinematic_fanuc_m_20ia(robot, T)

%Initialize q, 8 possible solutions are generally feasible
%Iniciamos q. hay 8 posibles soluciones
q=zeros(6,8);

%Evaluate the parameters
%Evaluamos los par�metros
d = eval(robot.DH.d);

%See geometry at the reference for this robot
%Ver geometr�a en la referencia para este robot
L6=d(6);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
%Calcule la posici�n de la mu�eca, siendo W el componente Z del sistema del efector final
Z6 = T(1:3,3);

% Pm: wrist position
% Pm: posici�n de la mu�eca
Pm = [Px Py Pz]' - L6*Z6; 

%first joint, two possible solutions admited: 
% if q(1) is a solution, then q(1) + pi is also a solution
%Primera articulaci�n, Hay dos posibles soluciones:
% si q(1) es una posible solucion, entonces q(1) + pi es tambien lo es
q1=atan2(Pm(2), Pm(1))-pi/2;


%Solve for q2:
%Resolvemos q2 (2 soluciones):
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);

q2_2=solve_for_theta2(robot, [q1+pi 0 0 0 0 0 0], Pm);

%Solve for q3:
%Resolvemos q3 (2 soluciones):
q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);

q3_2=solve_for_theta3(robot, [q1+pi 0 0 0 0 0 0], Pm);




% There exist two more possible solutions for the last three joints, 
%generally called wrist up and wrist down solutions. For this reason, 
%the next matrix doubles each column. For each two columns, two different
%configurations for theta4, theta5 and theta6 will be computed. These
%configurations are generally referred as wrist up and wrist down solution
%Existen dos soluciones m�s posibles para las �ltimas tres articulaciones, 
%generalmente llamadas soluciones de mu�eca arriba y mu�eca abajo. Por esta raz�n, 
%la siguiente matriz duplica cada columna. Para cada dos columnas, se computar�n 
%dos configuraciones diferentes para theta4, theta5 y theta6. Estas configuraciones 
%generalmente se denominan soluci�n de mu�eca arriba y mu�eca abajo.
q = [q1         q1         q1        q1       q1+pi   q1+pi   q1+pi   q1+pi;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0];

%Leave only the real part of the solutions
%Solo dejamos la parte real de las soluciones
q=real(q);

%Normalize q to [-pi, pi]
%Normalizamos q a [-pi, pi]
q(1,:) = normalize(q(1,:));
q(2,:) = normalize(q(2,:));

% Solve for the last three joints
% For any of the possible combinations (theta1, theta2, theta3)
% Resolvemos las �ltimas tres articulaciones
% Para cualquiera de las combinaciones posibles (theta1, theta2, theta3)
for i=1:2:size(q,2),
    qtemp = solve_spherical_wrist(robot, q(:,i), T, 1,'geometric'); %wrist up (mu�eca arriba)
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i)=qtemp;
    %Usamos el m�todo geometrico, pero tambi�n se puede usar el algebraico
        
    qtemp = solve_spherical_wrist(robot, q(:,i), T, -1, 'geometric'); %wrist down (mu�eca abajo)
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i+1)=qtemp;
    %Usamos el m�todo geometrico, pero tambi�n se puede usar el algebraico
end

 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution

%Para resolver la segunda articulaci�n theta2, se 
%devuelven dos soluciones diferentes, correspondientes 
%a la soluci�n del codo arriba y abajo.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)

%Evaluate the parameters
%Evaluamos los par�metros
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry
%Vemos la geometr�a
L2=a(2);
L3=d(4);

%There is a desfase in the join 3
%Existe un desfase en la articulaci�n 3
A3=a(3); 

%Given q1 is known, compute first D-H transformation
%Dado que q1 es conocido, calculamos la primera transformaci�n de D-H
T01=dh(robot, q, 1);

%Eslab�n equivalente para simplificar el desfase (PIT�GORAS)
l3 = sqrt (A3^2 + L3^2);

%Express Pm in the reference system 1
%Expresa Pm en el sistema de referencia 1
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(-p1(2), p1(1));
gamma = (acos((L2^2+r^2-l3^2)/(2*r*L2)));

if ~isreal(gamma)
    disp('WARNING:inversekinematic_fanuc_mate: the point is not reachable for this configuration, imaginary solutions'); 
    %gamma = real(gamma);
end

%return two possible solutions
%Devuelve dos posibles soluciones
q2(1) = pi/2 - beta - gamma; %elbow up (CODO ARRIBA)
q2(2) = pi/2 - beta + gamma; %elbow down (CODO ABAJO)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution

%Para resolver la tercera articulaci�n theta3, se 
%devuelven dos soluciones diferentes, correspondientes 
%a la soluci�n del codo arriba y abajo.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(robot, q, Pm)

%Evaluate the parameters
%Evaluamos los par�metros
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry
%Vemos la geometr�a
L2=a(2);
L3=d(4);

%There is a desfase in the join 3
%Existe un desfase en la articulaci�n 3
A3= a(3);

%Eslab�n equivalente para simplificar el desfase (PIT�GORAS)
l3 = sqrt(A3^2 + L3^2);

%the angle phi is fixed
%el �ngulo phi es fijo
phi=acos((A3^2+l3^2-L3^2)/(2*A3*l3));

%given q1 is known, compute first DH transformation
%Dado que q1 es conocido, calculamos la primera transformaci�n de D-H
T01=dh(robot, q, 1);

%Express Pm in the reference system 1
%Expresa Pm en el sistema de referencia 1
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

eta = (acos((L2^2 + l3^2 - r^2)/(2*L2*l3)));

if ~isreal(eta)
   disp('WARNING:inversekinematic_fanuc_mate: the point is not reachable for this configuration, imaginary solutions'); 
   %eta = real(eta);
end

%return two possible solutions
%Devuelve dos posibles soluciones
q3(1) = pi - phi - eta; %elbow up (CODO ARRIBA)
q3(2) = pi - phi + eta; %elbow down (CODO ABAJO)


%remove complex solutions for q for the q1+pi solutions
%Eliminar soluciones complejas para q para las soluciones q1 + pi
function  qreal = arrange_solutions(q)
qreal=q(:,1:4);

%sum along rows if any angle is complex, for any possible solutions, then v(i) is complex
%sumar a lo largo de las filas si alg�n �ngulo es complejo, para cualquier posible soluci�n, entonces v (i) es complejo
v = sum(q, 1);

for i=5:8,
    if isreal(v(i))
        %store the real solutions
        %almacenar las soluciones reales
        qreal=[qreal q(:,i)]; 
    end
end

qreal = real(qreal);