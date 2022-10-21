clear all
close all
clc

folder_name= 'C:\Users\vtn2\Documents\MATLAB - Local\Safety Impact Feedback\Calibration';
cd(folder_name);
num_files = size(dir([folder_name '/*.csv']),1);
Files = dir(fullfile(folder_name,'*.csv'));

trial = 1;
speed_num = 1;
X=[];
Y=[];
Z=[];
J = [];

for k = 1:1:num_files
    
    filename = Files(k).name;
    
    data = readtable(filename);
    data_IO = table2array(data(:,end-2:end-1));
    data = table2array(data(:,1:end-2));
    
    forces = data(:,20:25); current = data(:,14:19);
    time = data(:,1);time = (time - time(1))*1e-3; %Convert to seconds
    poses = data(:,8:13); q = data(:,2:7);
    curr1_filt = data(:,32);

    begin_time = time(1); end_time = time(end);
    begin_time = .5; end_time = time(end);
    
    ind = find(time>=begin_time&time<=end_time);
    time = time(ind,:);q = q(ind,:);curr1_filt = curr1_filt(ind,:);
    
    
    %%Med filt helps filter out impact!
    sig = curr1_filt;
    curr_ave(speed_num,trial) = mean(sig);
    speed(speed_num,1) = str2double(filename(6));
    
    ind_start = 50; %corresponds to removing 0.5 seconds of transience
    X = [X;time(ind_start:end)];
    Y = [Y;sig(ind_start:end)];
    Z = [Z;ones(length(time)-ind_start+1,1)*speed(speed_num,1)];

    length(time(ind_start:end))
    
    J = [J;q(ind_start:end,1)];
    
    if trial < 3
        trial = trial + 1;
    else
        trial = 1;
        speed_num = speed_num+1;
    end
end

[mdl_lin, gof_lin] = fit([speed;speed;speed],[curr_ave(:,1);curr_ave(:,2);curr_ave(:,3)],'poly1');
[mdl_inv, gof_inv] = fit([curr_ave(:,1);curr_ave(:,2);curr_ave(:,3)],[speed;speed;speed],'poly1');

figure;
plot(speed,curr_ave,'ko','markersize',5,'markerfacecolor','k');hold on;
plot(speed,mdl_lin(speed),'b','linewidth',2);
xlim([0,10]);ylim([0 2]);
xlabel('Speed (deg/sec)');ylabel('Steady State Current (amp)');
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);

figure;
plot(curr_ave,speed,'ko','markersize',5,'markerfacecolor','k');hold on;
plot([curr_ave(:,1);curr_ave(:,2);curr_ave(:,3)],mdl_inv([curr_ave(:,1);curr_ave(:,2);curr_ave(:,3)]),'b','linewidth',2);
ylim([0,10]);xlim([0 2]);
ylabel('Speed (deg/sec)');xlabel('Steady State Current (amp)');
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);

cd('C:\Users\vtn2\Documents\MATLAB - Local\Safety Impact Feedback');

mdl = fitrnet([X,Y,J], Z,'LayerSizes', 10,'Activations', 'relu','Lambda', 0,'IterationLimit', 1000,'Standardize', true);

save('Model_NN','mdl');
save('Model_Targ','mdl_lin');
return

inputTable = array2table([X,Y,J], 'VariableNames', {'column_1', 'column_2', 'column_3'});
predictorNames = {'column_1', 'column_2', 'column_3'};predictors = inputTable(:, predictorNames);
response = Z;
isCategoricalPredictor = [false, false, false];

regressionNeuralNetwork = fitrnet(predictors, response,'LayerSizes', 10,'Activations', 'relu','Lambda', 0,'IterationLimit', 1000,'Standardize', true);
predictorExtractionFcn = @(x) array2table(x, 'VariableNames', predictorNames);
neuralNetworkPredictFcn = @(x) predict(regressionNeuralNetwork, x);
trainedModel.predictFcn = @(x) neuralNetworkPredictFcn(predictorExtractionFcn(x));

