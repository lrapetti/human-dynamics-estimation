close all;
clear all;
clc;

%% Plot parameters
fontSize = 20;
lineWidth = 2;

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');
% plotSuffix = "HandEstimatesInLinkOrientation";
% plotSuffix = " - Expressed in Link Frame";=
% plotSuffix = " - Measurements in World Orientation & Estimates in Link Orientation";

%% Sum of offsetRemovedWrench measurements

sumOfOffsetRemovedMeasurements = data.wrenchEstimates(1:6,:)' + data.wrenchEstimates(13:18,:)' + data.wrenchEstimates(25:30,:)' + data.wrenchEstimates(37:42,:)';
sumOfEstimatedWrenchInLinkFrame = data.wrenchEstimates(7:12,:)' + data.wrenchEstimates(19:24,:)' + data.wrenchEstimates(31:36,:)' + data.wrenchEstimates(43:48,:)';
sumOfEstimatedWrenchInBaseFrame = data.wrenchEstimates(49:54,:)' + data.wrenchEstimates(61:66,:)' + data.wrenchEstimates(73:78,:)' + data.wrenchEstimates(85:90,:)';
sumOfEstimatedWrenchInWorldFrame = data.wrenchEstimates(55:60,:)' + data.wrenchEstimates(67:72,:)' + data.wrenchEstimates(79:84,:)' + data.wrenchEstimates(91:96,:)';

% % figure;
% % plot(sumOfOffsetRemovedMeasurements);
% % hold on;
% % title('sumOfOffsetRemovedMeasurements');
% % legend('$f_x [N]$', '$f_y [N]$', '$f_z [N]$','$m_x [Nm]$', '$m_y [Nm]$', '$m_z [Nm]$', 'Interpreter', 'latex', 'FontSize', 12);
% % 
% % figure;
% % plot(sumOfEstimatedWrenchInLinkFrame);
% % hold on;
% % title('sumOfEstimatedWrenchInLinkFrame');
% % legend('$f_x [N]$', '$f_y [N]$', '$f_z [N]$','$m_x [Nm]$', '$m_y [Nm]$', '$m_z [Nm]$', 'Interpreter', 'latex', 'FontSize', 12);

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
wrenchSourceName = {'Left Foot Wrench In Link Frame', 'Right Foot Wrench In Link Frame', 'Left Hand Wrench In Link Frame', 'Right Hand Wrench In Link Frame'};
momentumLegendString = ["$\dot{H}_{L_x}$", "$\dot{H}_{L_y}$", "$\dot{H}_{L_z}$", "$\dot{H}_{\omega_x}$", "$\dot{H}_{\omega_y}$", "$\dot{H}_{\omega_z}$"];


%% offSetRemovedMeasurement Vs Estimates

numberOfWrenchSources = 4;

for i = 1:numberOfWrenchSources
    
    fH = figure('units','normalized','outerposition',[0 0 1 1]);
    
    for s = 1:6
        
        subplot(2,3,s);
        plot(data.wrenchEstimates(s + 2 * 6 * (i-1),:)', 'LineWidth', lineWidth);
        hold on;
        plot(data.wrenchEstimates(6 + s + 2 * 6 * (i-1),:)', 'LineWidth', lineWidth, 'LineStyle', '--');
        hold on;
        xlabel('Samples', 'FontSize', fontSize);
        ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
        set (gca, 'FontSize' , fontSize)
        legend('Measured Wrench', 'Estimated Wrench', 'FontSize', fontSize, 'Location', 'Best');
        
    end
    
    a = axes;
    t = title (wrenchSourceName(i));
    t.FontSize = fontSize;
    a.Visible = 'off' ;
    t.Visible = 'on' ;
    
    %% Save figure
    save2pdf(strcat(wrenchSourceName(i) + ".pdf"), fH,300);
    
end


properRateOfChangeOfMomentumInWorldFrame = data.comProperAccelerationInWorldFrame;
properRateOfChangeOfMomentumInBaseFrame = data.comProperAccelerationInBaseFrame;

%% %% Proper Rate of Change of Momentum In Base Frame Vs Sum of External Wrenches In Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6 
    
    subplot(2,3,s);
    plot(properRateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumOfEstimatedWrenchInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;    
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Proper Rate of Change of Momentum - Sum of External Wrenches In Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

% % % % % % save2pdf("rateOfMomentumVsWrenchesInBaseFrame.pdf", fH,300);


% % %% %% Proper Rate of Change of Momentum In World Frame Vs Base Frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6 
% %     
% %     subplot(2,3,s);
% %     plot(properRateOfChangeOfMomentumInWorldFrame(s,:)', 'LineWidth', lineWidth);
% %     hold on;
% %     plot(properRateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;    
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(strcat("World - " + momentumLegendString(s)), strcat("Base - " + momentumLegendString(s)), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("Proper Rate of Change of Momentum In World Frame Vs Base Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % save2pdf("rateOfMomentumInWorldVsInBase.pdf", fH,300);

% % %% Import task1 simulatedy vector
% % task1simulatedy = importdata('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/task1simulatedy.txt');
% % leftHandSimulatedWrenchEstimates = task1simulatedy(:,169:174);
% % rightHandSimulatedWrenchEstimates = task1simulatedy(:,235:240);
% % rightFootSimulatedWrenchEstimates = task1simulatedy(:,313:318);
% % leftFootSimulatedWrenchEstimates = task1simulatedy(:,379:384);
% % sumOfEstimatedWrenchInLinkFrameInBaseFrame = task1simulatedy(:,403:408);
% % 
% % %% Comparison from the simulated y and the input y for task1
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6 
% %     
% %     subplot(2,3,s);
% %     plot(properRateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumOfEstimatedWrenchInLinkFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;    
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(strcat("Base - " + momentumLegendString(s)), strcat("Base - " + wrenchLegendString(s)), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end

% % %% Import task1 diff in y inout and simulatedy vector
% % task1Diff = importdata('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/task1DifferenceInyd.txt');
% % leftHandSimulatedWrenchEstimatestask1Diff = task1Diff(:,169:174);
% % rightHandSimulatedWrenchEstimatestask1Diff = task1Diff(:,235:240);
% % rightFootSimulatedWrenchEstimatestask1Diff = task1Diff(:,313:318);
% % leftFootSimulatedWrenchEstimatestask1Diff = task1Diff(:,379:384);
% % sumOfEstimatedWrenchInLinkFrameInBaseFrametask1Diff = task1Diff(:,403:408);
% % 
% % diffWrenches = [leftFootSimulatedWrenchEstimatestask1Diff  rightFootSimulatedWrenchEstimatestask1Diff leftHandSimulatedWrenchEstimatestask1Diff rightHandSimulatedWrenchEstimatestask1Diff];
% % 
% % %% Comparison from the simulated y and the input y for task1
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6 
% %     
% %     subplot(2,3,s);
% %     plot(sumOfEstimatedWrenchInLinkFrameInBaseFrametask1Diff(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;    
% %     xlabel('Samples', 'FontSize', fontSize);
% %     ylim([-100 100]); 
% %     
% % end
% % 
% % a = axes;
% % t = title ("Difference between proper rate of change of momentum input and estimated sum of external wrench estimates in Base Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % % t.Visible = 'on' ;

% % numberOfWrenchSources = 4;
% % 
% % for i = 1:numberOfWrenchSources
% %     
% %     fH = figure('units','normalized','outerposition',[0 0 1 1]);
% %     
% %     for s = 1:6
% %         
% %         subplot(2,3,s);
% %         plot(diffWrenches(:, 6 * (i-1) + s), 'LineWidth', lineWidth);
% %         hold on;
% %         xlabel('Samples', 'FontSize', fontSize);
% %         ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
% %         set (gca, 'FontSize' , fontSize)
% %         legend(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %         
% %     end
% %     
% %     a = axes;
% %     t = title (wrenchSourceName(i));
% %     t.FontSize = fontSize;
% %     a.Visible = 'off' ;
% %     t.Visible = 'on' ;
% %     
% % end