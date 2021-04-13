classdef DroneImage
    
    %DRONEIMAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = 'private', GetAccess = 'public')
        mI %Grauwertbild
        tanAlphaWidth {mustBeNumeric} %Seitenverhältnis Bildbreite zu Flughöhe
        tanAlphaHeight {mustBeNumeric} %Seitenverhältnis Bildhöhe zu Flughöhe
        droneHeight {mustBeNumeric} %Flughöhe der Drohne in Meter
        centerLatitude {mustBeNumeric} %Breitengrad in Grad
        centerLongitude {mustBeNumeric} %Längengrad in Grad
        phiNorth {mustBeNumeric} %Nordausrichtung im Bild in Grad
        kmPerDegLatitude {mustBeNumeric} %Lokale Umrechnungsfaktoren von Grad in Meter
        kmPerDegLongitude {mustBeNumeric} %Lokale Umrechnungsfaktoren von Grad in Meter
    end
    
    properties (Constant)
        camAngle = 45; %Kamerawinkel in Grad
    end
    
    properties (SetAccess = 'private', GetAccess = 'private')
        grenzLats {mustBeNumeric} %größter und kleinster Längengrad
        grenzLongs {mustBeNumeric} %größter und kleinster Breitengrad
        widthOfImgInM {mustBeNumeric}
        heightOfImgInM {mustBeNumeric}
        image %Eingelesenes Bild
    end
    
    methods (Access = 'private')
        function [dearKoords, dearGeoKoords] = recogObj(obj)
            Im = obj.image;
            
            rmat = Im(:,:,1); %Rote Bildebene
            gmat = Im(:,:,2); %Grüne Bildebene
            bmat = Im(:,:,3); %Blaue Bildebene
            
            levelr = 0.593; %Intensität des Rotanteils, muss abhängig vom Bild angepasst werden
            levelg = 0.505; %Intensität des Grünanteils, muss abhängig vom Bild angepasst werden
            levelb = 0.485; %Intensität des Blauanteils, muss abhängig vom Bild angepasst werden
            i1 = im2bw(rmat, levelr); %Rote Bildebene in schwarz/weiß umgewandelt
            i2 = im2bw(gmat, levelg); %Grüne Bildebene in schwarz/weiß umgewandelt
            i3 = im2bw(bmat, levelb); %Blaue Bildebene in schwarz/weiß umgewandelt
            Isum = (i1&i2&i3); %Alle drei Bildebenen, in schwarz/weiß umgewandelt und aufsummiert
            
            
            Icomp = imcomplement(Isum); %Invertiert Farben, Objekte werden nun in weiß und der Hintergrund in schwarz dargestellt
            Ifilled = Icomp;
            
            se = strel('disk', 20); %Gibt an in welcher Art und Weise die Formen erkannt werden sollen
            Iopenned = imopen(Ifilled, se); %Versucht die Formen zu finden/erkennen, und zu vervollständigen,
            %indem es z.b. erkennt dass es sich um einen Kreis handelt und diesen ausfüllt,
            %falls dieser Löcher beinhaltet, welche durch die Bildverabeitung enstanden sind.
            %Entfernt zu dem mögliches Bildrauschen
            
            Iregion = regionprops(Iopenned, 'centroid'); %Berechnet das geometrische Zentrum der erkannten Objekte/Formen
            [labeled, numObjects] = bwlabel(Iopenned, 4); %Speichert Bezeichnung und Anzahl an Objekten
            stats = regionprops(labeled, 'Eccentricity', 'Area', 'BoundingBox'); %Speichert Fläche, Koordinaten und Größe der, des Objekts, umgebenden Box und der Exzentrizität
            areas = [stats.Area]; %Speichert Fläche des Objekts
            eccentricities = [stats.Eccentricity]; %Speichert Exzentrizität (Abstand, bei einer Ellipse, eines Brennpunkts zum Mittelpunkt)
            
            idxOfSkittles = find(eccentricities); %Liefert alle Werte zurück welche nicht 0 sind, im eccentricities-Array
            statsDefects = stats(idxOfSkittles); %Liefert zwischengespeicherte Werte und Statistiken für das Objekt zurück
            
            %rectMittelPkt = {[]};
            
            %hold on;
            for idx = 1 : length(idxOfSkittles)
                h = rectangle('Position', statsDefects(idx).BoundingBox); %Erzeugt ein Rechteck rund um die erkannten Objekte
                crds = h.Position; %4-Vector mit der Position und Größe des Rechtecks
                
                width=crds(3); %Länge des Rechtecks in Pixel
                height=crds(4); %Breite des Rechtecks in Pixel
                xKoor=crds(1);
                yKoor=crds(2);
                
                dearKoords{idx} = [(xKoor + width/2), (yKoor + height/2)];
                
                [vLatitude, vLongitude] = obj.getGeographicalCoordinate((xKoor + width/2), (yKoor + height/2));
                %rectMittelPkt{idx} = [(xKoor + width/2) (yKoor + height/2)];
                dearGeoKoords{idx} = [vLatitude, vLongitude];
                %disp(dearKoords {idx}(2));
                
                %entfBildRectMittl = sqrt(power(bildMittelPkt(1)-rectMittelPkt(1),2)+power(bildMittelPkt(2)-rectMittelPkt(2),2));
                %entfBildRectMitteInCm{i}{idx} = (entfBildRectMittl*lengthOfOnePixelInMM)/10;
                %widthOfRectInCm = round(width*lengthOfOnePixelInMM/10,1); %Berechnet die Länge des Rechtsecks, in Zentimeter
                %heightOfRectInCm = round(height*lengthOfOnePixelInMM/10,1); %Berechnet die Breite des Rechtecks, in Zentimeter
                %rectSizeInCm{i}{idx} = [widthOfRectInCm heightOfRectInCm]; %Speichert Größe jedes erkannten Objekts (Recheck um Objekt), in Zentimer
                hold on;
            end
            
            %bilderNeu{i} = Im; %Speichert das jeweils verwendete Bild
            %amountObjects{i} = numObjects; %Speichert Anzahl an erkannten Objekten von Bildern
            %hold off;
        end
        
        function [peilWinkel, entfernungInPx] = calcPeilwinkel(obj)
            [rows, columns, numberOfColorChannels] = size(obj.mI); %Bildauflösung rows = Pixel in vertikale Richtung, columns = Pixel in horizontale Richtung
            
            bildMittelPkt = [columns/2, rows/2]; %Bildmittelpunkt mit Pixel-Koordinate definiert
            
            bildMitteObenPkt = [columns/2, 1]; %Punkt horizontal in der Mitte, jedoch an oberen Kante vom Bild (y = 1), in Pixel
            mitteObenVec = [bildMitteObenPkt(1)-bildMittelPkt(1) bildMitteObenPkt(2)-bildMittelPkt(2)]; %Vektor vom Bildmittelpunkt, vertikal zur oberen Bildkante
            
            linksObenPkt = [1 1]; %Linkes, oberes Eck des Bildes
            mitteLoVec = [linksObenPkt(1)-bildMittelPkt(1) linksObenPkt(2)-bildMittelPkt(2)]; %Vektor vom Bildmittelpunkt zum linken, oberen Eck
            
            cosTheta = max(min(dot(mitteLoVec,mitteObenVec)/(norm(mitteLoVec)*norm(mitteObenVec)),1),-1);
            thetaInDegrees = real(acosd(cosTheta)); %Winkel zwischen mitteLoVec-Vektor und mitteObenVec-Vektor
            
            %disp("Angle "+thetaInDegrees);
            
            mitteLoAngle = 360 - thetaInDegrees; %Himmelswinkel des mitteLoVec-Vektors
            mitteRoAngle = thetaInDegrees; %Himmelswinkel vom Bildmittelpunkt zum rechten oberen Eck
            mitteLuAngle = mitteLoAngle - 90; %Himmelswinkel vom Bildmittelpunkt zum linken unteren Eck
            mitteRuAngle = 90 + mitteRoAngle; %Himmelswinkel vom Bildmittelpunkt zum rechten unteren Eck
            
            peilWinkel = [mitteLoAngle mitteRoAngle mitteLuAngle mitteRuAngle];
            %disp(peilWinkel);
            entfernungInPx = norm(mitteLoVec); %Entfernung von Bildmittelpunkt zum rechten, oberen Eck in Pixel
            %disp(entfernungInPx);
        end
        
        function pktKoords = calcPktCoords(obj, peilWinkel, entfernungInM)
            startKoords = [obj.centerLatitude obj.centerLongitude];
            
            ha_1 = 6378137.0; %grosse Halbachse = Ellipsoid_Equatorialradius
            c = entfernungInM / ha_1;
            %disp(c);
            if (startKoords(1) >= 0)
                a = (90.0 - startKoords(1)) * pi / 180.0;
            else
                a = startKoords(1) * pi / 180.0;
            end
            
            q = (360.0 - peilWinkel) * pi / 180;
            b = acosd(cosd(q) * sind(a) * sind(c) + cosd(a) * cosd(c));
            latZiel = 90 - ((b(1) * 180) / pi);
            
            if (latZiel > 90)  %Suedhalbkugel - 180 Grad abziehen
                latZiel = latZiel - 180;
            end
            
            if (a==0 || b(1)==0)
                g = 0;
            else
                arg = (cosd(c) - cosd(a) * cosd(b(1))) / (sind(a) * sind(b(1)));
                if (arg <= -1)
                    g = pi;
                elseif (arg < 1)
                    g = acosd(arg);
                else
                    g = 0;
                end
            end
            
            if (peilWinkel <= 180)
                g = -1 * g;
            end
            
            lonZiel = (startKoords(2) - g * 180 / pi);
            
            pktKoords = [latZiel lonZiel];
        end
        function publishToMqtt(obj, dearGeoKoords)
            myMQTT = mqtt("tcp://iotmqtt.htl-klu.at", "Port", 1883, "Username", "htl-IoT", "Password", "iot..2015", "ClientID", "3432235defsdf"); %creates a new MQTT
            topic = "htl/dimplomarbeit/matlab/dearPos";
            mySub = subscribe(myMQTT,topic);
            
            pubString = "";
            
            for i=1:length(dearGeoKoords)
                pubString = pubString + dearGeoKoords{i}(1)+"_"+dearGeoKoords{i}(2)+",";
            end
            
            publish(myMQTT, topic, pubString);
        end
    end
    
    methods
        function obj = DroneImage(imageName)
            %Bekommt Bildnamen übergeben und lädt das Bild, sowie
            %extrahiert und initialisiert die entsprechenden Properties
            
            obj.image = imread(imageName);
            %imshow(obj.image);
            
            latLngAnglHeightStr = strsplit(imageName, '_'); %Splittet den Namen der Dateien nach '_'
            tmp = strsplit(imageName, '.');
            endung = '.'+tmp(length(tmp));
            
            obj.droneHeight = str2double(latLngAnglHeightStr(1));
            %disp(obj.droneHeight);
            
            obj.phiNorth = str2double(latLngAnglHeightStr(2));
            %disp(obj.phiNorth);
            
            obj.centerLatitude = str2double(latLngAnglHeightStr(3)); %Nimmt den ersten Teil des gesplitteten Strings, und wandelt diesen in einen double um
            %disp(obj.centerLatitude);
            
            obj.centerLongitude = str2double(strrep(latLngAnglHeightStr(4),endung,'')); %Nimmt den zweiten Teil des gesplitteten Strings, entfernt die gebliebene Endung ".jpg" und wandelt diesen in einen double um
            %disp(obj.centerLongitude);
            
            [rows, columns, numberOfColorChannels] = size(obj.image); %Bildauflösung rows = Pixel in vertikale Richtung, columns = Pixel in horizontale Richtung
            cameraResolution = [columns rows]; %Bildauflösung als 1x2-Vector
            
            diagonalPixels = sqrt((cameraResolution(1)^2)+(cameraResolution(2)^2)); %Berechnet die Anzahl an Pixel in der Bilddiagonale
            imgDiagonalLengthInM = tand(obj.camAngle/2)*obj.droneHeight*2; %Berechnet die Länge der Bilddiagonale, in Meter
            lengthOfOnePixelInMM = (imgDiagonalLengthInM/diagonalPixels)*1000; %Berechnet die Länge eines Pixels, in Millimeter
            %disp(lengthOfOnePixelInMM);
            
            obj.widthOfImgInM = cameraResolution(1)*(lengthOfOnePixelInMM/1000);
            %disp(widthOfImgInM);
            
            obj.heightOfImgInM = cameraResolution(2)*(lengthOfOnePixelInMM/1000);
            %disp(obj.heightOfImgInM);
            
            obj.tanAlphaWidth = obj.widthOfImgInM/obj.droneHeight;
            obj.tanAlphaHeight = obj.heightOfImgInM/obj.droneHeight;
            
            obj.image = imrotate(obj.image, -obj.phiNorth);
            obj.mI = rgb2gray(obj.image);
            
            [peilWinkel, entfernungInPx] = obj.calcPeilwinkel();
            %disp(entfernungInPx);
            
            entfernungInM = entfernungInPx * (lengthOfOnePixelInMM/1000);
            %disp(entfernungInM);
            maxLat = 0;
            maxLon = 0;
            minLat = Inf;
            minLon = Inf;
            for i=1:length(peilWinkel)
                pktKoords = obj.calcPktCoords(peilWinkel(i), entfernungInM);
                if pktKoords(1) > maxLat
                    maxLat = pktKoords(1);
                end
                if pktKoords(1)<minLat
                    minLat = pktKoords(1);
                end
                if pktKoords(2) > maxLon
                    maxLon = pktKoords(2);
                end
                if pktKoords(2) < minLon
                    minLon = pktKoords(2);
                end
            end
            
            obj.grenzLats = [minLat maxLat];
            obj.grenzLongs = [minLon maxLon];
            %disp(obj.grenzLongs(1));
            %disp(obj.grenzLongs(2));
            
            myMQTT = mqtt("tcp://iotmqtt.htl-klu.at", "Port", 1883, "Username", "htl-IoT", "Password", "iot..2015", "ClientID", "3432235defsdf"); %creates a new MQTT
            topic = "/htl/matlab/test";
            mySub = subscribe(myMQTT,topic);
            
            [dearKoords, dearGeoKoords] = obj.recogObj();
            obj.publishToMqtt(dearGeoKoords);
        end
        
        function showImage(obj, cType)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here [0 obj.widthOfImgInM 0 obj.heightOfImgInM]
            [rows, columns, numberOfColorChannels] = size(obj.mI);
            if cType == "deg"
                figure; imagesc([obj.grenzLongs(1) obj.grenzLongs(2)], [obj.grenzLats(1) obj.grenzLats(2)], obj.mI); colormap(gray); axis([obj.grenzLongs(1) obj.grenzLongs(2) obj.grenzLats(1) obj.grenzLats(2)]); axis tight; hold on;
                xlabel('Breitengrade')
                ylabel('Längengrade')
                title('Skalierung in Grad');
            elseif cType == "meter"
                figure; imagesc([0 obj.widthOfImgInM], [0 obj.heightOfImgInM],obj.mI); colormap(gray); axis([0 obj.widthOfImgInM 0 obj.heightOfImgInM]); axis tight; hold on;
                xlabel('Meter')
                ylabel('Meter')
                title('Skalierung in Meter');
            end
        end
        
        function [vLatitude, vLongitude] = getGeographicalCoordinate(obj,vX, vY)
            [rows, columns, numberOfColorChannels] = size(obj.mI);
            R = georasterref('LatitudeLimits', obj.grenzLats, 'LongitudeLimits', obj.grenzLongs,'RasterSize', [columns rows]);
            [lat, lon] = intrinsicToGeographic(R,vX,vY);
            vLatitude = lat;
            vLongitude = lon;
        end
        
        function markPoint(obj)
            %figure(1); imagesc([0 obj.widthOfImgInM], [0 obj.heightOfImgInM],obj.mI); colormap(gray); axis([0 obj.widthOfImgInM 0 obj.heightOfImgInM]); axis tight; hold on;
            
            [dearKoords, dearGeoKoords] = obj.recogObj();
            
            figure, imshow(obj.mI)
            for i=1:length(obj.recogObj())
                t = text(dearKoords{i}(1), dearKoords{i}(2), num2str(dearGeoKoords{i}(1)+" | "+dearGeoKoords{i}(2)),'Color','yellow'); %Zeigt Länge jedes Rechtecks an, in Zentimeter
                set(t);
            end
            
            while 1
                %[x y] = ginput(0);
                mouse_position=get(gca, 'CurrentPoint');
                x = mouse_position(1,1);
                y = mouse_position(1,2);
                [vLatitude, vLongitude] = obj.getGeographicalCoordinate(x,y);
                %viscircles([x y],2)
                gtext(num2str(vLatitude+"|"+vLongitude),'Color','yellow');
            end
        end
    end
end