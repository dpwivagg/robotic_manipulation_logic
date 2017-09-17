function gui
% GUI Select a data set from the pop-up menu, then
% click one of the plot-type push buttons. Clicking the button
% plots the selected data in the axes.

% Create and then hide the UI as it is being constructed
f = figure('Visible','off','Position',[360,500,900,570]);

% Construct the components
% hsurf   = uicontrol('Style','pushbutton',...
%             'String','Surf','Position',[315,220,70,25],...
%             'Callback',{@surfbutton_Callback});
% hmesh    = uicontrol('Style','pushbutton',...
%             'String','Mesh','Position',[315,180,70,25],...
%             'Callback',{@meshbutton_Callback});
% hcontour = uicontrol('Style','pushbutton',...
%             'String','Contour','Position',[315,135,70,25],...
%             'Callback',{@contourbutton_Callback});
htext    = uicontrol('Style','text',...
            'String','Select View','Position',[475,485,80,15]);
hviewController = uicontrol('Style','popupmenu',...
            'String',{'Iso','Top','Right','Front'},...
            'Position',[475,450,100,25],...
            'Callback',{@popup_menu_Callback});
% Add axes and define placement of live plot
ha = axes('Units','pixels','Position',[50,145,400,400]);
% Align components along their centers
%align([hsurf, hmesh, hcontour, htext, hviewController], 'Center','None');
align([htext, hviewController], 'Center','None');

% Initialize the UI.
% Change units to normalized so components resize automatically.
f.Units = 'normalized';
ha.Units = 'normalized';
% hsurf.Units = 'normalized';
% hmesh.Units = 'normalized';
% hcontour.Units = 'normalized';
htext.Units = 'normalized';
hviewController.Units = 'normalized';


% Create a plot in the axes.
threeLinkPlot(1, 1, eCoordinate(1, 1, 0, 0),...
    pCoordinate(1, 1, 1, 0, 0, 0 + 90));

% Assign a name to appear in the window title.
f.Name = 'Arm Controller GUI';

% Move the window to the center of the screen.
movegui(f,'center')

% Make the UI visible.
f.Visible = 'on';

%  Pop-up menu callback. Read the pop-up menu Value property to
%  determine which item is currently displayed and make it the
%  current data. This callback automatically has access to 
%  current_data because this function is nested at a lower level.
function popup_menu_Callback(source,eventdata) 
   % Determine the selected data set.
   str = source.String;
   val = source.Value;
   % Set current data to the selected data set.
   switch str{val};
   case 'Iso' % User selects Peaks.
      view(37.5, 30);
   case 'Top' % User selects Membrane.
      view(0,90);
   case 'Right' % User selects Sinc.
      view(90,0);
   case 'Front' % User selects
      view(0,0);
   end
end

% Push button callbacks. Each callback plots current_data in the
% specified plot type.
% function surfbutton_Callback(source,eventdata) 
%     % Display surf plot of the currently selected data.
%      surf(current_data);
% end
% 
% function meshbutton_Callback(source,eventdata) 
%     % Display mesh plot of the currently selected data.
%      mesh(current_data);
% end
% 
% function contourbutton_Callback(source,eventdata) 
%     % Display contour plot of the currently selected data.
%      contour(current_data);
% end
end