function DATA_Plot(dataplot,actuator_dynamics_on,type,roadnoise,u,ur,Y,Yr,w,Ts,t,tr);

%% data plot
% if dataplot == 1
%     
%     position = cumsum(Y(:,3))*Ts;
%     position =[0 ; position(1:end-1)];
%     position_r = cumsum(Yr(:,3))*Ts;
%     position_r =[0 ; position_r(1:end-1)];
%     
%     figure;
%     plot(t,position,'r','LineWidth',3)
%     hold on
%     plot(tr,position_r,'--b','LineWidth',3)
%     set(gca,'Fontsize',16,'FontWeight','demi')
%     xlabel('time(sec)')
%     ylabel('m')
%     title('x')
%     legend('w/ consideration','w/o consideration','Location','NorthWest')
%     set(gcf,'Position', [10 550 630 450])
%     grid on
% 
%     figure;
%     plot(t,Y(:,1)','r','LineWidth',3)
%     hold on
%     plot(tr,Yr(:,1)','--b','LineWidth',3)
%     set(gca,'Fontsize',16,'FontWeight','demi')
%     xlabel('time(sec)')
%     ylabel('m')
%     title('heave')
%     legend('w/ consideration','w/o consideration','Location','South')
%     set(gcf,'Position', [640 550 630 450])
%     grid on
% 
%     figure;
%     plot(t,Y(:,2),'r','LineWidth',3)
%     hold on
%     plot(tr,Yr(:,2),'--b','LineWidth',3)
%     set(gca,'Fontsize',16,'FontWeight','demi')
%     xlabel('time(sec)')
%     ylabel('[rad]')
%     title('pitch')
%     legend('w/ consideration','w/o consideration','Location','North')
%     set(gcf,'Position', [1270 550 630 450])
%     grid on
% 
%     figure;
%     plot(t,Y(:,3),'r','LineWidth',3)
%     hold on
%     plot(tr,Yr(:,3),'--b','LineWidth',3)
%     set(gca,'Fontsize',16,'FontWeight','demi')
%     xlabel('time(sec)')
%     ylabel('m/s')
%     title('dx')
%     legend('w/ consideration','w/o consideration','Location','NorthWest')
%     set(gcf,'Position', [10 50 630 450])
%     grid on
% 
%     figure;
%     plot(t,Y(:,4),'r','LineWidth',3)
%     hold on
%     plot(tr,Yr(:,4),'--b','LineWidth',3)
%     set(gca,'Fontsize',16,'FontWeight','demi')
%     xlabel('time(sec)')
%     ylabel('m/s')
%     title('heave velocity')
%     legend('w/ consideration','w/o consideration','Location','NorthEast')
%     set(gcf,'Position', [640 50 630 450])
%     grid on
% 
%     figure;
%     plot(t,Y(:,5),'r','LineWidth',3)
%     hold on
%     plot(tr,Yr(:,5),'--b','LineWidth',3)
%     set(gca,'Fontsize',16,'FontWeight','demi')
%     xlabel('time(sec)')
%     ylabel('[rad/s]')
%     title('pitch rate')
%     legend('w/ consideration','w/o consideration','Location','SouthEast')
%     set(gcf,'Position', [1270 50 630 450])
%     grid on
% 
%     if actuator_dynamics_on == 1
%         figure;
%         plot(t,Y(:,6)+Y(:,7),'r','LineWidth',3)
%         hold on
%         plot(tr,Yr(:,6)+Yr(:,7),'--b','LineWidth',3)
% %         plot(tr,Y(:,6)+Y(:,7)-(Yr(:,6)+Yr(:,7)),'--k','LineWidth',3)
%         set(gca,'Fontsize',16,'FontWeight','demi')
%         xlabel('time(sec)')
%         ylabel('N')
%         title('input(u)')
% %         legend('w/ consideration','w/o consideration','difference','Location','SouthWest')
%         legend('w/ consideration','w/o consideration','Location','SouthWest')    
%         set(gcf,'Position', [1950 550 630 450])
%         grid on
%     end
%     
%     if type == 4
%         figure;
%         plot(t,u(1,:)+u(2,:),'r','LineWidth',3)
%         hold on
%         plot(tr,ur(1,:)+ur(2,:),'--b','LineWidth',3)   
% %         plot(tr,u(1,:)+u(2,:)-(ur(1,:)+ur(2,:)),'--k','LineWidth',3)
%         set(gca,'Fontsize',16,'FontWeight','demi')
%         xlabel('time(sec)')
%         ylabel('voltage(V)')
%         title('input(V)')
% %         legend('w/ consideration','w/o consideration','difference','Location','SouthWest')    
%         legend('w/ consideration','w/o consideration','Location','SouthWest')    
%         set(gcf,'Position', [1950 50 630 450])
%         grid on
%     else
%         figure;
%         plot(t,u(1,:),'r','LineWidth',3)
%         hold on
%         plot(tr,ur(1,:),'--b','LineWidth',3)   
% %         plot(tr,u(1,:)-ur(1,:),'--k','LineWidth',3)
%         set(gca,'Fontsize',16,'FontWeight','demi')
%         xlabel('time(sec)')
%         ylabel('voltage(V)')
%         title('input(V)')
% %         legend('w/ consideration','w/o consideration','difference','Location','SouthWest')    
%         legend('w/ consideration','w/o consideration','Location','SouthWest')    
%         set(gcf,'Position', [1950 50 630 450])
%         grid on
%     end
%     
%     if roadnoise == 1
%         figure;
%         plot(t,w(1,:),'b','LineWidth',3)
%         set(gca,'Fontsize',16,'FontWeight','demi')
%         xlabel('time(sec)')
%         title('zuf')        
%         set(gcf,'Position', [2580 550 630 450])
%         grid on
%         
%         figure;
%         plot(t,w(2,:),'b','LineWidth',3)
%         set(gca,'Fontsize',16,'FontWeight','demi')
%         xlabel('time(sec)')
%         title('zur')        
%         set(gcf,'Position', [2580 50 630 450])
%         grid on
%     end
%         
% end

if dataplot == 1
    figure;
    set(gcf,'Position', [1930 50 630*3 950])
%     set(gcf,'Position', [1950 550 630 450])
    subplot(231)
    plot(t,Y(:,1)','r','LineWidth',3)
    hold on
    plot(tr,Yr(:,1)','--b','LineWidth',3)
    % set(gca,'Fontsize',16,'FontWeight','demi')
    xlabel('time(sec)')
    ylabel('m')
    title('heave')
    % legend('w/ consideration','w/o consideration','Location','South')
    grid on

    subplot(232)
    plot(t,Y(:,2),'r','LineWidth',3)
    hold on
    plot(tr,Yr(:,2),'--b','LineWidth',3)
    % set(gca,'Fontsize',16,'FontWeight','demi')
    xlabel('time(sec)')
    ylabel('[rad]')
    title('pitch')
    % legend('w/ consideration','w/o consideration','Location','North')
    grid on

    subplot(233)
    plot(t,Y(:,3),'r','LineWidth',3)
    hold on
    plot(tr,Yr(:,3),'--b','LineWidth',3)
    % set(gca,'Fontsize',16,'FontWeight','demi')
    xlabel('time(sec)')
    ylabel('m/s')
    title('dx')
    % legend('w/ consideration','w/o consideration','Location','NorthWest')
    grid on

    subplot(234)
    plot(t,Y(:,4),'r','LineWidth',3)
    hold on
    plot(tr,Yr(:,4),'--b','LineWidth',3)
    % set(gca,'Fontsize',16,'FontWeight','demi')
    xlabel('time(sec)')
    ylabel('m/s')
    title('heave velocity')
    % legend('w/ consideration','w/o consideration','Location','NorthEast')
    % set(gcf,'Position', [640 50 630 450])
    grid on

    subplot(235)
    plot(t,Y(:,5),'r','LineWidth',3)
    hold on
    plot(tr,Yr(:,5),'--b','LineWidth',3)
    % set(gca,'Fontsize',16,'FontWeight','demi')
    xlabel('time(sec)')
    ylabel('[rad/s]')
    title('pitch rate')
    % legend('w/ consideration','w/o consideration','Location','SouthEast')
    % set(gcf,'Position', [1270 50 630 450])
    grid on

    if actuator_dynamics_on == 1
        subplot(236)
        plot(t,Y(:,6)+Y(:,7),'r','LineWidth',3)
        hold on
        plot(tr,Yr(:,6)+Yr(:,7),'--b','LineWidth',3)
        % set(gca,'Fontsize',16,'FontWeight','demi')
        xlabel('time(sec)')
        ylabel('N')
        title('input(u)')
        % legend('w/ consideration','w/o consideration','Location','SouthWest')    
        % set(gcf,'Position', [1950 550 630 450])
        grid on
    else
        if type == 4
            subplot(236)
            plot(t,u(1,:)+u(2,:),'r','LineWidth',3)
            hold on
            plot(tr,ur(1,:)+ur(2,:),'--b','LineWidth',3)   
    %         plot(tr,u(1,:)+u(2,:)-(ur(1,:)+ur(2,:)),'--k','LineWidth',3)
    %         set(gca,'Fontsize',16,'FontWeight','demi')
            xlabel('time(sec)')
            ylabel('voltage(V)')
            title('input(V)')
    %         legend('w/ consideration','w/o consideration','difference','Location','SouthWest')    
    %         legend('w/ consideration','w/o consideration','Location','SouthWest')    
    %         set(gcf,'Position', [1950 50 630 450])
            grid on
        else
            subplot(236)
            plot(t,u(1,:),'r','LineWidth',3)
            hold on
            plot(tr,ur(1,:),'--b','LineWidth',3)   
    %         plot(tr,u(1,:)-ur(1,:),'--k','LineWidth',3)
    %         set(gca,'Fontsize',16,'FontWeight','demi')
            xlabel('time(sec)')
            ylabel('voltage(V)')
            title('input(V)')
    %         legend('w/ consideration','w/o consideration','difference','Location','SouthWest')    
    %         legend('w/ consideration','w/o consideration','Location','SouthWest')    
    %         set(gcf,'Position', [1950 50 630 450])
            grid on            
        end
    end

end





    