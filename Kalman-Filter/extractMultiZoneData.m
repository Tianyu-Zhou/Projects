%Austin Coffman
%example script to get multi zone pugh hall data:

%clear interface:
    clear
    close all
    clc
    
%pick file:
    dateName = '2013-01-28';
    fileOpenName = ['Input_', dateName,'_7_openloop.mat'];
    
%load file:
    dataLoad = load(fileOpenName);
    data = dataLoad.input_data;
    dataInfo = dataLoad.input_info;
    
%other properties:
    Ts = dataInfo.T; %sample time of data, in seconds.
    
%exogenous variables (same for all zones):
    Ta = data.u(:,5) + 273.15; %ambient air temperature (celsius).
    etaSol = data.u(:,6); %solar irradiance (KW/m^2).
    
%zone 1 data:
    zoneOneTZ = data.y(:,1); %zone one temperature (Celsius).
    zoneOneCO2 = data.w(:,1); %zone one q_{dist} (KW).
    zoneOneQHVAC = data.u(:,1); %zone one q_{hvac} (KW).
    
%zone 2 data:
    zoneTwoTZ = data.y(:,2); %zone two temperature (Celsius).
    zoneTwoCO2 = data.w(:,2); %zone two q_{dist} (KW).
    zoneTwoQHVAC = data.u(:,2); %zone two q_{hvac} (KW).
    
%zone 3 data:
    zoneThreeTZ = data.y(:,3); %zone Three temperature (Celsius).
    zoneThreeCO2 = data.w(:,3); %zone Three q_{dist} (KW).
    zoneThreeQHVAC = data.u(:,3); %zone Three q_{hvac} (KW).    

%zone 4 data:
    zoneFourTZ = data.y(:,4); %zone Four temperature (Celsius).
    zoneFourCO2 = data.w(:,4); %zone Four q_{dist} (KW).
    zoneFourQHVAC = data.u(:,4); %zone Four q_{hvac} (KW).    
    
%zone 5 data:
    zoneFiveTZ = data.y(:,5); %zone Five temperature (Celsius).
    zoneFiveCO2 = data.w(:,5); %zone Five q_{dist} (KW).

%zone 6 data:
    zoneSixTZ = data.y(:,6); %zone Six temperature (Celsius).
    zoneSixCO2 = data.w(:,6); %zone Six q_{dist} (KW).
    
%zone 7 data:
    zoneSevenTZ = data.y(:,7); %zone Seven temperature (Celsius).
    zoneSevenCO2 = data.w(:,7); %zone Seven q_{dist} (KW).
    
%zone 8 data:
    zoneEightTZ = data.y(:,8); %zone Eight temperature (Celsius).
    zoneEightCO2 = data.w(:,8); %zone Eight q_{dist} (KW).
    
%zone 9 data:
    zoneNineTZ = data.y(:,9); %zone Nine temperature (Celsius).
    zoneNineCO2 = data.w(:,9); %zone Nine q_{dist} (KW).
    
%zone 10 data:
    zoneTenTZ = data.y(:,10); %zone Ten temperature (Celsius).
    zoneTenCO2 = data.w(:,10); %zone Ten q_{dist} (KW).
    
    Tz = [zoneOneTZ,zoneTwoTZ,zoneThreeTZ,zoneFourTZ,zoneFiveTZ,zoneSixTZ,zoneSevenTZ,zoneEightTZ,zoneNineTZ,zoneTenTZ] + 273.15;    
    Qdist = [zoneOneCO2,zoneTwoCO2,zoneThreeCO2,zoneFourCO2,zoneFiveCO2,zoneSixCO2,zoneSevenCO2,zoneEightCO2,zoneNineCO2,zoneTenCO2];     
    Qhvac = [zoneOneQHVAC,zoneTwoQHVAC,zoneThreeQHVAC,zoneFourQHVAC];

    %Parameter estimates
    R0 = 7; 
    R1 = 48; 
    R2 = 48; 
    Cc = 48; 
    Cz = 25; 
    Cz2 = 25; 
    Ae = 25; 
    X0=[R0 R1 R2 Cc Cz Cz2 Ae]';
    
    Ts = 1/12; %Ts 1 hour to 5 mins
    N = 2016;

    TzInit = [285;285;285;285;285;285;285;285;285;285];       
    TwInit = [285;285;285;285;285];
    QdistInit = [0;0;0;0;0;0;0;0;0;0];
    
    nParams = 7;
    LB = 0.01*ones(nParams,1);
    UB = 50*ones(nParams,1);
    
    error_fun = @(p) kalmanfilter(p,Ta,etaSol,Qhvac,Ts,Tz',TzInit,TwInit,QdistInit,N);
    
    options = optimoptions('fmincon','Algorithm','interior-point');
    %Form of [X,FVAL] = fmincon(FUN,X0,A,B,Aeq,Beq,LB,UB,NONLCON)
    [X,FVAL] = fmincon(error_fun,X0,[],[],[],[],LB,UB,[],options);
    
    p = X;
    
    [Fval,x_hat_fin] = kalmanfilter(p,Ta,etaSol,Qhvac,Ts,Tz',TzInit,TwInit,QdistInit,N);
    
    for i=1:1:10
        figure();
        plot(Tz(:,i));
        hold on;
        plot(x_hat_fin(:,i));
        legend('Tz true','Tz estimate')
        title('Tz true VS Tz estimate');
        xlabel('Samples')
        ylabel('Kelvin temperature')
    end
    
    for i=1:1:10
        figure();
        plot(Qdist(:,i));
        hold on;
        plot(x_hat_fin(:,i+15));
        legend('Qdist true','Qdist estimate')
        title('Qdist true VS Qdist estimate');
        xlabel('Samples')
        ylabel('Power')
    end
    
    
    