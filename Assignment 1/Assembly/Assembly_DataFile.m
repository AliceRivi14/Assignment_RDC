% Simscape(TM) Multibody(TM) version: 7.5

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(5).translation = [0.0 0.0 0.0];
smiData.RigidTransform(5).angle = 0.0;
smiData.RigidTransform(5).axis = [0.0 0.0 0.0];
smiData.RigidTransform(5).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 0 50];  % mm
smiData.RigidTransform(1).angle = 0;  % rad
smiData.RigidTransform(1).axis = [0 0 0];
smiData.RigidTransform(1).ID = "RootGround[Link1 v2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [1000 2.8421709430399999e-13 0];  % mm
smiData.RigidTransform(2).angle = 0;  % rad
smiData.RigidTransform(2).axis = [0 0 0];
smiData.RigidTransform(2).ID = "SixDofRigidTransform[Link2 v4:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 0 -30];  % mm
smiData.RigidTransform(3).angle = 0;  % rad
smiData.RigidTransform(3).axis = [0 0 0];
smiData.RigidTransform(3).ID = "SixDofRigidTransform[Motor v3:4]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [0 0 0];  % mm
smiData.RigidTransform(4).angle = 0;  % rad
smiData.RigidTransform(4).axis = [0 0 0];
smiData.RigidTransform(4).ID = "SixDofRigidTransform[Base v3:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [1000 1.4210854715199999e-13 -30];  % mm
smiData.RigidTransform(5).angle = 0;  % rad
smiData.RigidTransform(5).axis = [0 0 0];
smiData.RigidTransform(5).ID = "SixDofRigidTransform[Motor v3:5]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(4).mass = 0.0;
smiData.Solid(4).CoM = [0.0 0.0 0.0];
smiData.Solid(4).MoI = [0.0 0.0 0.0];
smiData.Solid(4).PoI = [0.0 0.0 0.0];
smiData.Solid(4).color = [0.0 0.0 0.0];
smiData.Solid(4).opacity = 0.0;
smiData.Solid(4).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 1741.062289839753;  % lbm
smiData.Solid(1).CoM = [499.99999999999989 8.267334542461116e-10 149.99999999999997];  % mm
smiData.Solid(1).MoI = [26546004.036020245 213899006.95700559 214329076.64542934];  % lbm*mm^2
smiData.Solid(1).PoI = [-1.0476528599871488e-08 -2.6281149647817323e-08 -5.641207707626785e-08];  % lbm*mm^2
smiData.Solid(1).color = [0.62745098039215685 0.62745098039215685 0.62745098039215685];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Link1 v2.ipt_{998A5A72-4DE7-A0CA-15E6-E884C55BDE45}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 1923.8408368618755;  % lbm
smiData.Solid(2).CoM = [531.6456416091994 1.2489346800395871e-10 200];  % mm
smiData.Solid(2).MoI = [42101345.045752093 175009178.71377 162596914.3118926];  % lbm*mm^2
smiData.Solid(2).PoI = [-8.0588681537312096e-09 -2.6281149647817323e-08 0.00012748335463192388];  % lbm*mm^2
smiData.Solid(2).color = [1 1 1];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Link2 v4.ipt_{E6E635A5-46B4-3EE7-9C8C-63B76482A852}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 195.78387064309709;  % lbm
smiData.Solid(3).CoM = [-0.027770059950946377 -0.020827550843815948 230.06525965010073];  % mm
smiData.Solid(3).MoI = [5617636.7473018086 5617874.3958842326 1170641.5252557325];  % lbm*mm^2
smiData.Solid(3).PoI = [957.99303156097665 1277.3236296227976 -407.65655652576123];  % lbm*mm^2
smiData.Solid(3).color = [1 1 1];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Motor v3.ipt_{A31A8202-4625-E7BB-2226-6591127CF226}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 1543.1025100685933;  % lbm
smiData.Solid(4).CoM = [-229.35195732797197 1.5542189497456133e-10 200.00000000000003];  % mm
smiData.Solid(4).MoI = [139160774.56540346 33761199.425473571 128561386.5910683];  % lbm*mm^2
smiData.Solid(4).PoI = [5.1576756183849859e-08 3.9421724471725988e-08 -5.4876977532311466e-05];  % lbm*mm^2
smiData.Solid(4).color = [0.18823529411764706 0.23137254901960785 0.58823529411764708];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Base v3.ipt_{EE3C23AC-4F16-7F83-80F8-A2B139AE2A3A}";

