clear all;
close all;
tic;


fileID=fopen('prueba_def_2.txt','r');
data=(fscanf(fileID,'%u',[252 1024]))';
Time={};
Pheripherical_temperature= {};
Internal_temperature = {};
Physical_Activity ={};

for i=0:1019
    day=string(data(i+1,1));
    month=string(data(i+1,2));
    year=string(bitshift(data(i+1,3),8)+data(i+1,4));
    current_date=strcat(day,"/", month,"/", year);
    Date((i*31+1):(31+(31*i)),1) = {current_date};
end

for i=1:1020
    for j=5:8:252
        current_time=string( hours(data(i,j)) + minutes(data(i,j+1)) + seconds(data(i,j+2)),'hh:mm:ss');      
        Time = [Time;current_time];
        current_temp_int=string((bitshift(data(i,j+3),8)+data(i,j+4))/100);
        Internal_temperature = [ Internal_temperature;current_temp_int];
        current_temp_phe=string((bitshift(data(i,j+5),8)+data(i,j+6))/100);
        Pheripherical_temperature = [Pheripherical_temperature;current_temp_phe];
        if data(i,j+7)==0
            current_phy_act="NO";
        else
            current_phy_act="YES";
        end
        Physical_Activity =[Physical_Activity;current_phy_act];
    end
end

dataOut=table(Date,Time,Pheripherical_temperature,Internal_temperature,Physical_Activity);
filename = 'patientdata.xlsx';
writetable(dataOut,filename);
toc;