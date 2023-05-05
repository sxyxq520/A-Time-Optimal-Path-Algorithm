classdef RobotObj < handle
 
    
    % public properties
    properties
        ModelName; % model name of of robot
        r;  % struct contains robot parameters
        bnd_ball;
    end
    methods
        % constructor
        function obj = RobotObj(model)
            switch model
                case 'ER80_2600' % M20iA robot model from ARTE
                    obj = obj.buildER80_2600;
                otherwise
                    error('Robot model not supported yet.');
            end
            obj.ModelName=model;
        end
       
        % build M20iA model
        function obj = buildER80_2600(obj)
            % useful inline functions
%             % skew matrix for cross product
%             hat=@(w)[0      -w(3)   w(2);...
%              w(3)   0       -w(1);...
%              -w(2)  w(1)    0];
            % from [Ixx Iyy Izz Ixy Iyz Ixz] to I matrix
            v2M=@(x)[x(1)   x(4)    x(6);...
                     x(4)   x(2)    x(5);...
                     x(6)   x(5)    x(3)];
            
            obj.r.name = 'ER80_2600';
            obj.r.manufacturer = '埃斯顿';
            obj.r.n = 6;
            % important: should set path before run
            obj.r.path = which('inversekinematic_fanuc_m_20ia.m');
            obj.r.path=obj.r.path(1:end-31);
            [obj.r.cad.base{1}.f,obj.r.cad.base{1}.v]=...
                stlread([obj.r.path,'link0_exp.stl']); 
            obj.r.cad.base{1}.color = [0.4,0.4,0.4];
            for j=1:6
                [obj.r.cad.link{j}.f,obj.r.cad.link{j}.v]=...
                    stlread([obj.r.path,'link',num2str(j),'_exp.stl']); 
            end
            for j=1:5
                obj.r.cad.link{j}.color = 'y';
            end
            obj.r.cad.link{6}.color = 'k';

            % DH parameters
            obj.r.Theta0  = [pi/2 -pi/2 0 0 0 0];  % jnt1 use different value than ARTE                                    % Joint variable offset (rad)
            obj.r.D       = [0.356 0 0.071887 1.464252 0 0.497];     %六轴d6高度0.505                             % Joint extension (m)
            obj.r.A       = [0 0.496827 0.609368 0 0 0];                                  % Joint offset (m)
            obj.r.Alpha   = [0 -pi/2 0 -pi/2 pi/2 -pi/2 ];                               % Joint twist (rad)
            obj.r.Qlim    = [-178 178; -148 65; -66 190; ...
                        -180 180; -118 118; -360 360]'*pi/180; % modified jnt angle limit of jnt2                     % Joint angle limit (rad)
            obj.r.QDlim   = [-100 100; -105 105; -110 110; ...
                        -215 215; -165 165; -205 205]'*pi/180;
                    
            obj.r.T_B2W = eye(4);

            % Reducer parameters
            obj.r.G       = [136 120 100 100 50 50];           %未改                            % Gear ratio 齿轮齿数比
            
            % Link parameters, ignoring any friction for now
            obj.r.Ml = [262.544 97.886 115.559 32.834 10.118 1.002];         % Link mass (kg)
            obj.r.Rl = [0.2908     0.0117    0.0179;
                        0.5008	   0.0001	 0.3369;
                        0.0002	   0.5810    0.0298;
                        0.0001    -0.0657   -0.1496;
                       -0.0003     0.0793    0.0050;
                       -0.0002       0      -0.0110].';      % Link center of gravity relative to each joint's frame (m)
  %                                     from [Ixx Iyy Izz Ixy Iyz Ixz] to I matrix           
            linkInertia = [9.5370  41.5389  44.6226  6.5103  -1.3168  -0.2375;
                          13.2915  42.1303 	30.1011  0.0048   0.0005  16.9631;
                          51.3931  1.0989   51.2056  0.0120   1.5548  0.0031;                   
                           1.8192   1.6257   0.4390 -0.0004   0.1201  -0.0005;
                           0.1277   0.0374   0.1307  0.0001   0.0001  -0.0001;
                           0.0013	0.0013	 0.0022	   0       0	     0]; %铰链惯性%未改
            for j=1:6
                obj.r.Jlmat(:,:,j) = v2M(linkInertia(j,:)); % in link frame
                obj.r.Jlmat(:,:,j)=Inertia_parallel_axis_toCog(...
                    obj.r.Jlmat(:,:,j),obj.r.Ml(j),-obj.r.Rl(:,j)); % in cog frame
            end
            
            % Gravity
            obj.r.gravity     = [0; 0; 9.81];                                        % Gravity vector in world coordinates (m/sec^2)
            
            % some common poses
            obj.r.qz  = [0 -90 0 0 0 0]*pi/180;         % zero angles, L shaped pose零度角
            obj.r.qh  = [0 -90 0 0 0 0]*pi/180;         % home pose原位置
            
            % setup bounding balls for collision detection 设置碰撞检测的边界球
            obj.bnd_ball.c{1}=[0 0 0.1];
            obj.bnd_ball.r{1}=0.3;
            
            obj.bnd_ball.c{2}(1,:)=[0,-0.05,0.054];
            obj.bnd_ball.r{2}(1)=0.33;
            obj.bnd_ball.c{2}(2,:)=[0.5,0,0];
            obj.bnd_ball.r{2}(2)=0.22;
            
            obj.bnd_ball.c{3}(1,:)=[0.61,0,0.4];
            obj.bnd_ball.r{3}(1)=0.3;
            obj.bnd_ball.c{3}(2,:)=[0.1,0,0.4];
            obj.bnd_ball.r{3}(2)=0.2;
            
            obj.bnd_ball.c{4}(1,:)=[0,0,0];
            obj.bnd_ball.r{4}(1)=0.27;
            obj.bnd_ball.c{4}(2,:)=[0,0.4,0];
            obj.bnd_ball.r{4}(2)=0.22;
            obj.bnd_ball.c{4}(3,:)=[0,0.8,0];
            obj.bnd_ball.r{4}(3)=0.22;
            
            obj.bnd_ball.c{5}(1,:)=[0,0,0];
            obj.bnd_ball.r{5}(1)=0.20;
            obj.bnd_ball.c{5}(2,:)=[0,0,-0.31];
            obj.bnd_ball.r{5}(2)=0.20;
            
            obj.bnd_ball.c{6}(1,:)=[0 0.495 0];
            obj.bnd_ball.r{6}(1)=0.02;
            
            obj.bnd_ball.c{7}=[];
            obj.bnd_ball.r{7}=[];
            
%             obj.bnd_ball.c{7}(1,:)=[0,0,-0.2];
%             obj.bnd_ball.r{7}(1)=0.06; 
%             obj.bnd_ball.c{7}(2,:)=[0.07,-0.095,-0.08];
%             obj.bnd_ball.r{7}(2)=0.08;
%             obj.bnd_ball.c{7}(3,:)=[0.07,0.095,-0.08];
%             obj.bnd_ball.r{7}(3)=0.08;
%             obj.bnd_ball.c{7}(4,:)=[-0.07,-0.095,-0.08];
%             obj.bnd_ball.r{7}(4)=0.08;
%             obj.bnd_ball.c{7}(5,:)=[-0.07,0.095,-0.08];
%             obj.bnd_ball.r{7}(5)=0.08;
%             obj.bnd_ball.c{7}(6,:)=[0,0,-0.0115];
%             obj.bnd_ball.r{7}(6)=0.0115;
        end 
        
        % analytical inverse kinematics of M20iA
        % using DH parameter from ARTE, except Theta0
        % current implementation cannot handle exceptions
        % input: R, P: tool frame orientation and position
        % output: q = joint position
        function [ theta ] = kinv_ER80_2600( obj,R,P )
        % initialize as row vector
        theta=nan(1,6);
        % DH parameters
        alpha=obj.r.Alpha;
        A=obj.r.A;
        D=obj.r.D;
        offset=obj.r.Theta0;
        % Tool frame
        if isfield(obj.r,'tool')
            R_tool=tool(1:3,1:3);
            T_tool=tool(1:3,4);
        else
            R_tool=eye(3);
            T_tool=[0;0;0];
        end
        %
        P = P(:);
        S6=[R,P;zeros(1,3),1]/[R_tool,T_tool;zeros(1,3),1];
        p=S6(1:3,4);
        R6=S6(1:3,1:3);
        w6=[0;0;D(6)];
        c=p-R6*w6; %末端4的矩阵
        
 nx = R6(1, 1); ox = R6(1, 2); ax = R6(1, 3); px = c(1);
 ny = R6(2, 1); oy = R6(2, 2); ay = R6(2, 3); py = c(2);
 nz = R6(3, 1); oz = R6(3, 2); az = R6(3, 3); pz = c(3)-0.358;
syms a1 a2 d3 d4;
 a1=0.496827;a2=0.609368;d3=0.071887;d4=1.464252;%机器人关节尺寸
 %角度1的解
        theta(1)=atan2(py,px)-atan2(d3,sqrt(px^2+py^2-d3^2));
  c1=cos(theta(1));s1=sin(theta(1));
        
  %角度3的解
        m=px^2+py^2+pz^2+a1^2-2*a1*c1*px-2*a1*s1*py;
k=(m-a2^2-d4^2-d3^2)/(2*a2);
%第一组解
theta(3)=-atan2(k,(sqrt(d4^2-k^2)));
c3_1=cos(theta(3));s3_1=sin(theta(3));

%         c0=c;
%         c=[c(1)-A(1)*cos(theta(1));...
%            c(2)-A(1)*sin(theta(1));...
%            c(3)-D(1)];
%         Lc=norm(c);
%         L2=sqrt(A(3)^2+D(4)^2);
%         Lc0=sqrt((A(2)+A(3))^2+D(4)^2);
%         theta(3)=acos((L2^2+A(2)^2-Lc^2)/(2*L2*A(2)))-acos((L2^2+A(2)^2-Lc0^2)/(2*L2*A(2)));
%         theta(3)=-theta(3);
        if ~isreal(theta(3))
            theta(3)=nan;
            return;
        end
%         dir=[cos(theta(1)),sin(theta(1))];
%         temp=dot(dir,[c0(1),c0(2)]);
%         if temp>A(1)
%             theta(2)=acos((A(2)^2+Lc^2-L2^2)/(2*A(2)*Lc))+atan(c(3)/sqrt(c(1)^2+c(2)^2));
%         elseif temp<A(1)
%             theta(2)=acos((A(2)^2+Lc^2-L2^2)/(2*A(2)*Lc))-atan(c(3)/sqrt(c(1)^2+c(2)^2))+pi;
%         else
%             theta(2)=acos((A(2)^2+Lc^2-L2^2)/(2*A(2)*Lc))+pi/2;
%         end
        %计算角度2
%第一组解
theta(2)=atan2(-a2*c3_1*pz+(c1*px+s1*py-a1)*(a2*s3_1-d4),(-d4+a2*s3_1)*pz+(c1*px+s1*py-a1)*a2*c3_1)-theta(3);
c23_1=cos(theta(2))*cos(theta(3))-sin(theta(2))*sin(theta(3));
s23_1=sin(theta(2))*cos(theta(3))+cos(theta(2))*sin(theta(3));

        if ~isreal(theta(2))
            theta(2)=nan;
            return;
        end
%         theta(2)=-theta(2)-obj.r.Theta0(2);
        q=theta;
%         for i=1:3
%             q(i)=q(i)+offset(i);
%         end
        TCP_T=eye(4);
        for i=1:3    
            TCP_T=TCP_T*...
                [         cos(q(i))          -sin(q(i))                     0                A(i)     ;...
                 sin(q(i))*cos(alpha(i))    cos(q(i))*cos(alpha(i))     -sin(alpha(i))   -D(i)*sin(alpha(i));...
                 sin(q(i))*sin(alpha(i))    cos(q(i))*sin(alpha(i))      cos(alpha(i))   D(i)*cos(alpha(i));...
                             0                      0                       0                   1];
        end
        T456=TCP_T\S6;
        %计算角度4对应四个解
     %第一组解
theta(4)=atan2((-ax*s1+ay*c1),(-ax*c1*c23_1-ay*s1*c23_1+az*s23_1));
c4_1=cos(theta(4));s4_1=sin(theta(4));
        
        if ~isreal(theta(4))
            theta(4)=nan;
            return;
        end
        
        %计算角度5对应四个解
    %第一组解
theta(5)=atan2((-ax*(c1*c23_1*c4_1+s1*s4_1)-ay*(s1*c23_1*c4_1-c1*s4_1)+az*(s23_1*c4_1)),(ax*(-c1*s23_1)-ay*s1*s23_1+az*(-c23_1)));
c5_1=cos(theta(5));s5_1=sin(theta(5));

 if ~isreal(theta(5))
            theta(5)=nan;
            return;
 end
        
 %计算角度6对应的四个解
%第一组解
s6_1=-nx*(c1*c23_1*s4_1-s1*c4_1)-ny*(s1*c23_1*s4_1+c1*c4_1)+nz*s23_1*s4_1;
c6_1=nx*((c1*c23_1*c4_1+s1*s4_1)*c5_1-c1*s23_1*s5_1)+ny*((s1*c23_1*c4_1-c1*s4_1)*c5_1-s1*s23_1*s5_1)-nz*(s23_1*c4_1*c5_1+c23_1*s5_1);
theta(6)=atan2(s6_1,c6_1);

 if ~isreal(theta(6))
            theta(6)=nan;
            return;
 end
%         theta(5)=acos(T456(3,3));
%         if sin(theta(5))~=0
%             theta(4)=atan2(-T456(2,3)/sin(theta(5)),-T456(1,3)/sin(theta(5)));
%             theta(6)=atan2(-T456(3,2)/sin(theta(5)),T456(3,1)/sin(theta(5)));
%         elseif cos(theta(5))==1 % singuality
%             theta(4)=nan;
%             theta(5)=nan;
%         end

        end

        
        % hand crafted newton euler, for casadi purpose
        function [ Torque ] = NewtonEuler( obj, Pos, Vel, Acc )
            % transfer variables
            n=obj.r.n;
            Torque=[];
            gravity=obj.r.gravity(:);
            q=Pos(:); %+obj.r.Theta0(:);% add zero, pos vector
            qd=Vel(:);% vel vector
            qdd=Acc(:);% acc vector
            % define temp variables
            ROT=cell(1,n);
            PSTAR=cell(1,n);
            OMEGA=cell(1,n);
            OMEGADOT=cell(1,n);
            ACC=cell(1,n);
            ACC_COG=cell(1,n);
            F=cell(1,n); %force
            Nt=cell(1,n); %torque
            % forward recursion 
            for j=1:n
                % rotation & translation for each link
                ROT{j}=[cos(q(j)), -sin(q(j)), 0;...
                     sin(q(j))*cos(obj.r.Alpha(j)), cos(q(j))*cos(obj.r.Alpha(j)),  -sin(obj.r.Alpha(j));...
                     sin(q(j))*sin(obj.r.Alpha(j)), cos(q(j))*sin(obj.r.Alpha(j)), cos(obj.r.Alpha(j))];
                PSTAR{j}=[obj.r.A(j),-obj.r.D(j)*sin(obj.r.Alpha(j)),obj.r.D(j)*cos(obj.r.Alpha(j))].';
                % omega & omegadot
                if j==1
                    t1=[0,0,qd(j)].';
                    t3=[0,0,qdd(j)].';
                else
                    t1=OMEGA{j-1}+[0,0,qd(j)].';
                    t3=OMEGADOT{j-1}+[0,0,qdd(j)].'+...
                        cross(OMEGA{j-1},[0,0,qd(j)].');
                end
                OMEGA{j}=ROT{j}.'*t1;
                OMEGADOT{j}=ROT{j}.'*t3;
                % acc
                ACC{j}=cross(OMEGADOT{j},PSTAR{j})+...
                    cross(OMEGA{j},cross(OMEGA{j},PSTAR{j}));
                if j==1
                    t1=ROT{j}.'*gravity;
                else
                    t1=ROT{j}.'*ACC{j-1};
                end
                ACC{j}=ACC{j}+t1;
                % abar
                ACC_COG{j}=cross(OMEGADOT{j},obj.r.Rl(:,j))+...
                    cross(OMEGA{j},cross(OMEGA{j},obj.r.Rl(:,j)))+...
                    ACC{j};
            end
            % backward recursion 
            for j=n:-1:1
                % F
                t4=ACC_COG{j}*obj.r.Ml(j);
                if j~=n
                    F{j}=t4+ROT{j+1}*F{j+1};
                else
                    F{j}=t4;
                end
                % Nt
                t1=cross(PSTAR{j}+obj.r.Rl(:,j),t4);
                if j~=n
                    t1=ROT{j+1}*(...
                    cross(ROT{j+1}.'*PSTAR{j},F{j+1})+...
                        Nt{j+1}...
                    )+t1;
                end
                Nt{j}=t1+obj.r.Jlmat(:,:,j)*OMEGADOT{j}+...
                    cross(OMEGA{j},obj.r.Jlmat(:,:,j)*OMEGA{j});
            end
            % compute torque total for each axis
            for j=1:n
                t=dot(Nt{j},ROT{j}.'*[0,0,1].');
                % (currently ignore friction)
                Torque=[Torque;t];
            end 
        end
        
    end
    
end

