im = DroneImage("2.2_285_33.124_12.323.jpeg");
im.showImage("deg");
im.showImage("meter");
im.markPoint();


%textlist=dir(['C:\Users\KRAVOS\Documents\MATLAB\' '*.jpeg']); %Speichern der Namen aller .jpg-Dateien
%for k=1:length(textlist)
    %disp(textlist(k).name);
    %im = DroneImage(string([textlist(k).name]));
    %im.showImage();
%end